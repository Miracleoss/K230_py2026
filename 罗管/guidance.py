"""
guidance.py - 制导律Python实现
完全移植自 dart_guidance C++ 代码
包含：四元数运算、坐标变换、视线角计算、比例导引律
"""

import math

# ==================== 类型定义 ====================

class Quaternion:
    """四元数结构"""
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"Quaternion({self.w:.6f}, {self.x:.6f}, {self.y:.6f}, {self.z:.6f})"


class Cordation:
    """三维坐标"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"Cordation({self.x:.6f}, {self.y:.6f}, {self.z:.6f})"


class GuidanceInput:
    """制导输入数据结构"""
    def __init__(self):
        self.pixel_dx = 0.0       # x方向像素差
        self.pixel_dy = 0.0       # y方向像素差
        self.q = Quaternion(1.0, 0.0, 0.0, 0.0)  # 姿态四元数
        self.target_velocity = 0.0  # 弹目接近速度 (m/s)
        self.timestamp = 0          # 时间戳（毫秒）


class GuidanceOutput:
    """制导输出数据结构"""
    def __init__(self):
        self.accel_pitch = 0.0    # 俯仰方向加速度 (m/s²)
        self.accel_yaw = 0.0      # 偏航方向加速度 (m/s²)
        self.valid = False        # 输出是否有效


class GuidanceConfig:
    """系统配置结构"""
    def __init__(self):
        self.nav_ratio = 4.0      # 导航比 N'
        self.camera_matrix = [    # 相机内参矩阵
            500.0, 0.0, 320.0,
            0.0, 500.0, 240.0,
            0.0, 0.0, 1.0
        ]
        self.filter_alpha = 0.3   # 低通滤波器系数
        self.max_accel_yaw = 100.0    # 偏航最大加速度 (m/s²)
        self.max_accel_pitch = 100.0  # 俯仰最大加速度 (m/s²)


# ==================== 数学工具函数 ====================

def clamp_float(value, min_val, max_val):
    """限制值在指定范围内"""
    if value < min_val:
        return min_val
    if value > max_val:
        return max_val
    return value


def lerp_float(a, b, t):
    """线性插值"""
    t = clamp_float(t, 0.0, 1.0)
    return a + (b - a) * t


def safe_atan2(y, x):
    """安全的atan2计算"""
    if abs(x) < 1e-7 and abs(y) < 1e-7:
        return 0.0
    return math.atan2(y, x)


def quaternion_norm(q):
    """计算四元数模长"""
    return math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)


def quaternion_normalize(q):
    """四元数归一化"""
    norm = quaternion_norm(q)
    if norm > 1e-7:
        return Quaternion(
            w=q.w / norm,
            x=q.x / norm,
            y=q.y / norm,
            z=q.z / norm
        )
    # 如果模长为0，返回单位四元数
    return Quaternion(1.0, 0.0, 0.0, 0.0)


def quaternion_conjugate(q):
    """四元数共轭"""
    return Quaternion(q.w, -q.x, -q.y, -q.z)


def quaternion_multiply(a, b):
    """四元数乘法"""
    result = Quaternion()
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    return result


def quaternion_relative(current, reference):
    """计算相对四元数（当前姿态相对于基准姿态）"""
    # 1. 求基准四元数的共轭（逆）
    ref_conj = quaternion_conjugate(reference)
    # 2. 基准的逆 放前面，当前的 放后面
    return quaternion_multiply(ref_conj, current)


def euler_to_quaternion(roll, pitch, yaw):
    """
    欧拉角转四元数 (Z-Y-X顺序: yaw -> pitch -> roll)
    
    Args:
        roll: 滚转角 (弧度)
        pitch: 俯仰角 (弧度)
        yaw: 偏航角 (弧度)
    
    Returns:
        Quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def low_pass_filter(input_val, prev, alpha):
    """一阶低通滤波器"""
    alpha = clamp_float(alpha, 0.0, 1.0)
    return prev + alpha * (input_val - prev)


def angle_difference(angle1, angle2):
    """计算角度差（考虑周期边界）"""
    diff = angle1 - angle2
    # 将角度差限制在[-π, π]范围内
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


def quaternion_to_euler_zyx(q):
    """
    从四元数计算欧拉角（Z-Y-X顺序：yaw->pitch->roll）
    
    Returns:
        (roll, pitch, yaw) 元组，单位为弧度
    """
    # 归一化四元数
    q = quaternion_normalize(q)
    
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


# ==================== Guidance 类 ====================

class Guidance:
    """制导律主类 - 完全移植自 C++ Guidance 类"""

    def __init__(self):
        # 参数结构体
        self.g_input = GuidanceInput()
        self.g_config = GuidanceConfig()

        # 角速度计算参数
        self.pre_timestamp = 0
        self.prev_los_angle_yaw = 0.0
        self.prev_los_angle_pitch = 0.0
        self.prev_los_angle_yaw_velocity = 0.0
        self.prev_los_angle_pitch_velocity = 0.0

        # 基础计算参数
        self.reference_quat = Quaternion(1.0, 0.0, 0.0, 0.0)

        # 初始化确认符
        self.is_fire = False
        self.initialized = False
        self.reference_set = False
        self.config_initialized = False

    def guidance_init(self, config, ref_qw, ref_qx, ref_qy, ref_qz):
        """
        初始化制导系统
        
        Args:
            config: GuidanceConfig 配置参数
            ref_qw, ref_qx, ref_qy, ref_qz: 基准四元数分量
        """
        self.g_config = config
        self.config_initialized = True
        
        self.reference_quat.w = ref_qw
        self.reference_quat.x = ref_qx
        self.reference_quat.y = ref_qy
        self.reference_quat.z = ref_qz
        
        # 归一化基准四元数
        self.reference_quat = quaternion_normalize(self.reference_quat)
        
        # 初始化状态
        self.initialized = True
        self.reference_set = True
        self.is_fire = False
        self.pre_timestamp = 0
        self.prev_los_angle_yaw = 0.0
        self.prev_los_angle_pitch = 0.0
        
        # 初始化输入
        self.g_input.pixel_dx = 0.0
        self.g_input.pixel_dy = 0.0
        self.g_input.q = Quaternion(1.0, 0.0, 0.0, 0.0)
        self.g_input.target_velocity = 0.0
        self.g_input.timestamp = 0


    def fire(self):
        """更新发射标志位"""
        self.is_fire = True

    def guidance_update(self, dx, dy, qw, qx, qy, qz, target_velocity, timestamp):
        """
        更新视觉数据和姿态数据
        
        Args:
            dx: x方向像素差（目标x - 图像中心x）
            dy: y方向像素差（目标y - 图像中心y）
            qw, qx, qy, qz: 四元数分量
            target_velocity: 弹目接近速度 (m/s)
            timestamp: 时间戳（毫秒）
        """
        # 更新输入数据
        self.g_input.pixel_dx = dx
        self.g_input.pixel_dy = dy
        self.g_input.timestamp = timestamp

        self.g_input.q.w = qw
        self.g_input.q.x = qx
        self.g_input.q.y = qy
        self.g_input.q.z = qz

        self.g_input.q = quaternion_normalize(self.g_input.q)
        
        self.g_input.target_velocity = target_velocity

    def guidance_calculate(self):
        """
        计算制导指令（主函数）
        
        Returns:
            GuidanceOutput: 制导输出，包含俯仰和偏航方向的加速度指令
        """
        if not self.is_fire:
            output = GuidanceOutput()
            output.valid = False
            return output
        
        if not self.initialized or not self.reference_set:
            output = GuidanceOutput()
            output.valid = False
            return output
        
        # 检查输入数据是否有效
        if self.g_input.timestamp == 0:
            output = GuidanceOutput()
            output.valid = False
            return output
        
        # ===== 核心算法 =====
        
        # 1. 像素坐标系 -> 相机坐标系
        camera = Cordation()
        camera.x = (self.g_input.pixel_dx - self.g_config.camera_matrix[2]) / self.g_config.camera_matrix[0]
        camera.y = (self.g_input.pixel_dy - self.g_config.camera_matrix[5]) / self.g_config.camera_matrix[4]
        camera.z = 1.0
        
        # 2. 相机坐标系 -> 镖体系
        dart = Cordation()
        dart.x = camera.z
        dart.y = camera.x
        dart.z = camera.y
        
        # 3. 镖体系 -> 惯性系
        rel_q = quaternion_relative(self.g_input.q, self.reference_quat)
        
        # 从四元数计算旋转矩阵（弹体到惯性系）
        C00 = 1.0 - 2.0 * (rel_q.y * rel_q.y + rel_q.z * rel_q.z)
        C01 = 2.0 * (rel_q.x * rel_q.y - rel_q.w * rel_q.z)
        C02 = 2.0 * (rel_q.x * rel_q.z + rel_q.w * rel_q.y)
        
        C10 = 2.0 * (rel_q.x * rel_q.y + rel_q.w * rel_q.z)
        C11 = 1.0 - 2.0 * (rel_q.x * rel_q.x + rel_q.z * rel_q.z)
        C12 = 2.0 * (rel_q.y * rel_q.z - rel_q.w * rel_q.x)
        
        C20 = 2.0 * (rel_q.x * rel_q.z - rel_q.w * rel_q.y)
        C21 = 2.0 * (rel_q.y * rel_q.z + rel_q.w * rel_q.x)
        C22 = 1.0 - 2.0 * (rel_q.x * rel_q.x + rel_q.y * rel_q.y)
        
        # 使用旋转矩阵将镖体系坐标转换到惯性系
        inertial = Cordation()
        inertial.x = C00 * dart.x + C01 * dart.y + C02 * dart.z
        inertial.y = C10 * dart.x + C11 * dart.y + C12 * dart.z
        inertial.z = C20 * dart.x + C21 * dart.y + C22 * dart.z
        
        # 4. 计算视线角
        los_angle_yaw = safe_atan2(inertial.y, inertial.x)
        los_angle_pitch = safe_atan2(-inertial.z, math.sqrt(inertial.x * inertial.x + inertial.y * inertial.y))
        
        # 5. 检查时间戳是否合理（防止除零错误）
        if self.pre_timestamp == 0:
            # 第一次计算：记录当前时间戳和视线角，返回零输出
            self.pre_timestamp = self.g_input.timestamp
            self.prev_los_angle_yaw = los_angle_yaw
            self.prev_los_angle_pitch = los_angle_pitch
            
            zero_output = GuidanceOutput()
            zero_output.valid = True
            return zero_output
        
        # 6. 计算时间差（毫秒转换为秒）
        dt = (self.g_input.timestamp - self.pre_timestamp) / 1000.0
        if dt <= 0.0 or dt > 1.0:
            output = GuidanceOutput()
            output.valid = False
            return output
        
        # 7. 计算视线角速度
        los_angle_yaw_velocity = (los_angle_yaw - self.prev_los_angle_yaw) / dt
        los_angle_pitch_velocity = (los_angle_pitch - self.prev_los_angle_pitch) / dt
        
        # 8. 应用低通滤波器平滑角速度
        los_angle_yaw_velocity = low_pass_filter(
            los_angle_yaw_velocity, 
            self.prev_los_angle_yaw_velocity, 
            self.g_config.filter_alpha
        )
        los_angle_pitch_velocity = low_pass_filter(
            los_angle_pitch_velocity, 
            self.prev_los_angle_pitch_velocity, 
            self.g_config.filter_alpha
        )
        
        # 9. 计算制导指令（比例导引）
        n_yaw = self.g_config.nav_ratio * self.g_input.target_velocity * los_angle_yaw_velocity
        n_pitch = self.g_config.nav_ratio * self.g_input.target_velocity * los_angle_pitch_velocity
        
        # 10. 限制加速度范围
        n_yaw = clamp_float(n_yaw, -self.g_config.max_accel_yaw, self.g_config.max_accel_yaw)
        n_pitch = clamp_float(n_pitch, -self.g_config.max_accel_pitch, self.g_config.max_accel_pitch)
        
        # 11. 更新历史数据
        self.prev_los_angle_yaw = los_angle_yaw
        self.prev_los_angle_pitch = los_angle_pitch
        self.prev_los_angle_pitch_velocity = los_angle_pitch_velocity
        self.prev_los_angle_yaw_velocity = los_angle_yaw_velocity
        self.pre_timestamp = self.g_input.timestamp
        
        # 12. 准备输出
        output = GuidanceOutput()
        output.accel_pitch = n_pitch
        output.accel_yaw = n_yaw
        output.valid = True
        
        return output

    def guidance_reset(self):
        """重置系统状态"""
        self.initialized = False
        self.reference_set = False
        self.is_fire = False
        self.pre_timestamp = 0
        self.prev_los_angle_yaw = 0.0
        self.prev_los_angle_pitch = 0.0
        
        # 重置输入
        self.g_input.pixel_dx = 0.0
        self.g_input.pixel_dy = 0.0
        self.g_input.q = Quaternion(1.0, 0.0, 0.0, 0.0)
        self.g_input.target_velocity = 0.0
        self.g_input.timestamp = 0
