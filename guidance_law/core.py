"""
制导核心算法模块

实现广义比例导引律和完整的制导系统。
基于发射前四元数确定基准平面，计算视线角，输出法向加速度指令。
"""

import time
import math
import numpy as np  
from .coordinate import (
    quaternion_to_euler_zyx,
    compute_relative_quaternion,
    quaternion_rotate,
    quaternion_conjugate
)
from .camera import CameraModel, estimate_distance_from_area
from .estimator import AttitudeEstimator
from .logger import DataLogger
from .utils import limit_value, calculate_derivative
from .config import get_default_config, validate_camera_params


class GuidanceSystem:
    """
    制导系统类
    
    实现完整的制导算法，包括：
    1. 基于发射前四元数的基准平面确定
    2. 像素差到视线角转换
    3. 距离估计（基于像素面积）
    4. 广义比例导引律计算
    5. 数据记录
    """
    
    def __init__(self, camera_params=None, N_prime=3.5, 
                 reference_area_at_1m=5000, output_limit=10.0,
                 log_dir="logs", log_prefix="guidance_log"):
        """
        初始化制导系统
        
        参数:
            camera_params: 相机参数字典
            N_prime: 导航比 (3-5之间)
            reference_area_at_1m: 1米距离时的参考像素面积
            output_limit: 输出限制 (±output_limit m/s²)
            log_dir: 日志目录
            log_prefix: 日志文件前缀
        """
        # 使用默认相机参数或自定义参数
        if camera_params is None:
            default_config = get_default_config()
            camera_params = default_config['camera']
        
        # 验证相机参数
        if not validate_camera_params(camera_params):
            raise ValueError("无效的相机参数")
        
        # 存储参数
        self.camera_params = camera_params
        self.N_prime = N_prime
        self.reference_area_at_1m = reference_area_at_1m
        self.output_limit = output_limit
        
        # 创建相机模型
        self.camera_model = CameraModel(
            focal_length=camera_params['focal_length'],
            pixel_size=camera_params['pixel_size'],
            optical_center_x=camera_params['optical_center_x'],
            optical_center_y=camera_params['optical_center_y'],
            image_width=camera_params['image_width'],
            image_height=camera_params['image_height']
        )
        
        # 创建姿态估计器
        self.attitude_estimator = AttitudeEstimator()
        
        # 创建数据记录器
        self.data_logger = DataLogger(log_dir=log_dir, log_prefix=log_prefix)
        
        # 状态变量
        self.reference_quaternion = None  # 发射前基准四元数
        self.is_initialized = False       # 是否已初始化（设置基准）
        
        # 历史数据（用于计算变化率）
        self.previous_angle_x = None
        self.previous_angle_y = None
        self.previous_time = None
        
        # 统计信息
        self.update_count = 0
        self.total_processing_time = 0.0
        
        # 启动数据记录
        self.data_logger.start_recording()
        
        print(f"制导系统已初始化:")
        print(f"  导航比 N' = {N_prime}")
        print(f"  输出限制 = ±{output_limit} m/s²")
        print(f"  参考面积@1m = {reference_area_at_1m} 像素")
    
    def set_reference_quaternion(self, qw, qx, qy, qz):
        """
        设置发射前的基准四元数
        
        参数:
            qw, qx, qy, qz: 基准四元数分量
        """
        self.reference_quaternion = (qw, qx, qy, qz)
        self.is_initialized = True
        
        # 初始化姿态估计器
        self.attitude_estimator.initialize_from_quaternion(qw, qx, qy, qz)
        
        print(f"基准四元数已设置: ({qw:.4f}, {qx:.4f}, {qy:.4f}, {qz:.4f})")
    
    def compute_line_of_sight_angle_relative(self, pixel_dx, pixel_dy, current_quaternion):
        """
        计算相对于基准平面的视线角
        
        参数:
            pixel_dx, pixel_dy: 像素差
            current_quaternion: 当前四元数 (qw, qx, qy, qz)
        
        返回:
            (angle_x, angle_y): 基准坐标系下的视线角（弧度）
        """
        if not self.is_initialized:
            raise ValueError("制导系统未初始化，请先设置基准四元数")
        
        # 1. 将像素差转换为机体坐标系下的角度
        angle_x_body, angle_y_body = self.camera_model.pixel_difference_to_angle(pixel_dx, pixel_dy)
        
        # 2. 计算相对四元数（从基准坐标系到当前坐标系）
        q_rel = compute_relative_quaternion(current_quaternion, self.reference_quaternion)
        
        # 3. 将机体坐标系下的角度向量转换到基准坐标系
        # 注意：q_rel是从基准坐标系到当前坐标系的旋转
        # 我们需要将当前机体坐标系下的角度转换到基准坐标系
        angle_vector_body = np.array([angle_x_body, angle_y_body, 0])
        
        # 使用相对四元数的逆（共轭）进行旋转
        q_rel_conj = quaternion_conjugate(q_rel)
        angle_vector_ref = quaternion_rotate(q_rel_conj, angle_vector_body)
        
        return angle_vector_ref[0], angle_vector_ref[1]
    
    def compute_guidance_command(self, pixel_dx, pixel_dy, pixel_area, 
                                current_quaternion=None, omega=None, dt=None):
        """
        计算制导指令
        
        参数:
            pixel_dx, pixel_dy: 像素差
            pixel_area: 像素面积（用于距离估计）
            current_quaternion: 当前四元数 (qw, qx, qy, qz) - 可选
            omega: 角速度 (omega_x, omega_y, omega_z) 弧度/秒 - 可选
            dt: 时间间隔（秒）- 可选，自动计算
        
        返回:
            (a_cmd_pitch, a_cmd_yaw): 俯仰和偏航通道的法向加速度指令 (m/s²)
        """
        start_time = time.time()
        
        # 更新时间间隔
        current_time = time.time()
        if dt is None:
            if self.previous_time is None:
                dt = 0.01  # 默认10ms
            else:
                dt = current_time - self.previous_time
        
        # 更新姿态估计
        if current_quaternion is not None:
            # 使用四元数更新姿态
            self.attitude_estimator.update_from_quaternion(*current_quaternion)
            current_q = current_quaternion
        elif omega is not None:
            # 使用角速度积分更新姿态
            self.attitude_estimator.update_from_gyro(*omega)
            current_q = self.attitude_estimator.get_quaternion()
        else:
            # 没有姿态信息，使用上一次的姿态
            current_q = self.attitude_estimator.get_quaternion()
        
        # 如果尚未初始化，返回零指令
        if not self.is_initialized:
            self.previous_time = current_time
            return 0.0, 0.0
        
        try:
            # 1. 计算相对于基准平面的视线角
            current_angle_x, current_angle_y = self.compute_line_of_sight_angle_relative(
                pixel_dx, pixel_dy, current_q
            )
            
            # 2. 计算视线角变化率
            if self.previous_angle_x is not None and self.previous_angle_y is not None:
                q_dot_x = calculate_derivative(current_angle_x, self.previous_angle_x, dt)
                q_dot_y = calculate_derivative(current_angle_y, self.previous_angle_y, dt)
            else:
                q_dot_x = 0.0
                q_dot_y = 0.0
            
            # 3. 估计弹目距离
            estimated_distance = estimate_distance_from_area(
                pixel_area, self.reference_area_at_1m
            )
            
            # 限制距离范围
            estimated_distance = limit_value(estimated_distance, 0.5, 50.0)
            
            # 4. 应用广义比例导引律
            # 公式: a_cmd = N' * R * q_dot
            a_cmd_pitch_raw = self.N_prime * estimated_distance * q_dot_y
            a_cmd_yaw_raw = self.N_prime * estimated_distance * q_dot_x
            
            # 5. 限制输出范围
            a_cmd_pitch = limit_value(a_cmd_pitch_raw, -self.output_limit, self.output_limit)
            a_cmd_yaw = limit_value(a_cmd_yaw_raw, -self.output_limit, self.output_limit)
            
            # 6. 获取欧拉角（用于记录）
            roll, pitch, yaw = self.attitude_estimator.get_euler_angles()
            
            # 7. 记录数据
            self.data_logger.log_guidance_data(
                timestamp=current_time,
                pixel_dx=pixel_dx,
                pixel_dy=pixel_dy,
                pixel_area=pixel_area,
                current_quaternion=current_q,
                reference_quaternion=self.reference_quaternion,
                angle_x=current_angle_x,
                angle_y=current_angle_y,
                q_dot_x=q_dot_x,
                q_dot_y=q_dot_y,
                estimated_distance=estimated_distance,
                a_cmd_pitch=a_cmd_pitch,
                a_cmd_yaw=a_cmd_yaw,
                dt=dt,
                is_initialized=self.is_initialized,
                euler_angles=(roll, pitch, yaw),
                omega=omega
            )
            
            # 8. 更新历史数据
            self.previous_angle_x = current_angle_x
            self.previous_angle_y = current_angle_y
            self.previous_time = current_time
            
            # 更新统计信息
            self.update_count += 1
            processing_time = time.time() - start_time
            self.total_processing_time += processing_time
            
            # 定期打印状态
            if self.update_count % 50 == 0:
                avg_time = self.total_processing_time / self.update_count
                print(f"制导更新 {self.update_count}: "
                      f"平均处理时间 {avg_time*1000:.1f}ms, "
                      f"指令: pitch={a_cmd_pitch:.2f} m/s², yaw={a_cmd_yaw:.2f} m/s²")
            
            return a_cmd_pitch, a_cmd_yaw
            
        except Exception as e:
            print(f"计算制导指令时出错: {e}")
            return 0.0, 0.0
    
    def update(self, pixel_dx, pixel_dy, pixel_area, 
              current_quaternion=None, omega=None, is_launch_detected=False):
        """
        更新制导系统（便捷方法）
        
        参数:
            pixel_dx, pixel_dy: 像素差
            pixel_area: 像素面积
            current_quaternion: 当前四元数 - 可选
            omega: 角速度 - 可选
            is_launch_detected: 是否检测到发射
        
        返回:
            (a_cmd_pitch, a_cmd_yaw): 制导指令
        """
        # 如果检测到发射且尚未初始化，设置基准四元数
        if is_launch_detected and not self.is_initialized:
            if current_quaternion is not None:
                self.set_reference_quaternion(*current_quaternion)
            else:
                print("警告：检测到发射但无四元数数据，无法设置基准")
        
        # 计算制导指令
        return self.compute_guidance_command(pixel_dx, pixel_dy, pixel_area,
                                           current_quaternion, omega)
    
    def get_status(self):
        """
        获取制导系统状态
        
        返回:
            dict: 状态信息字典
        """
        avg_processing_time = 0.0
        if self.update_count > 0:
            avg_processing_time = self.total_processing_time / self.update_count
        
        # 获取姿态估计器状态
        attitude_status = self.attitude_estimator.get_status()
        
        return {
            'initialized': self.is_initialized,
            'update_count': self.update_count,
            'avg_processing_time_ms': avg_processing_time * 1000,
            'reference_quaternion_set': self.reference_quaternion is not None,
            'is_recording': self.data_logger.is_recording,
            'record_count': self.data_logger.record_count,
            'attitude_estimator': attitude_status
        }
    
    def stop(self):
        """停止制导系统"""
        self.data_logger.stop_recording()
        print("制导系统已停止")
    
    def reset(self):
        """重置制导系统"""
        self.reference_quaternion = None
        self.is_initialized = False
        self.previous_angle_x = None
        self.previous_angle_y = None
        self.previous_time = None
        self.update_count = 0
        self.total_processing_time = 0.0
        
        self.attitude_estimator.reset()
        
        print("制导系统已重置")
    
    def __del__(self):
        """析构函数"""
        self.stop()


def generalized_proportional_navigation(R, q_dot, N_prime=3.5):
    """
    广义比例导引律
    
    参数:
        R: 弹目距离或接近速度因子
        q_dot: 视线角变化率
        N_prime: 导航比
    
    返回:
        法向加速度指令
    """
    return N_prime * R * q_dot


def calculate_line_of_sight_rate(current_angle, previous_angle, dt):
    """
    计算视线角变化率
    
    参数:
        current_angle: 当前视线角
        previous_angle: 上一时刻视线角
        dt: 时间间隔
    
    返回:
        视线角变化率
    """
    return calculate_derivative(current_angle, previous_angle, dt)
