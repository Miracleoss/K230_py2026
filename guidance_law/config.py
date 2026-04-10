"""
配置参数模块

提供制导系统的默认配置参数。
包括相机参数、制导参数、输出限制等。
"""

# 默认相机参数（基于常见相机配置）
DEFAULT_CAMERA_PARAMS = {
    'focal_length': 3.04,        # 焦距 (mm) - 典型值
    'pixel_size': 0.0014,        # 像元尺寸 (mm/像素) - 典型值
    'optical_center_x': 320.5,   # 光心X坐标 (像素) - 640x480图像的中心
    'optical_center_y': 240.5,   # 光心Y坐标 (像素)
    'image_width': 640,          # 图像宽度 (像素)
    'image_height': 480,         # 图像高度 (像素)
    'sensor_width': 4.8,         # 传感器宽度 (mm)
    'sensor_height': 3.6,        # 传感器高度 (mm)
}

# 高分辨率相机参数
HIGH_RES_CAMERA_PARAMS = {
    'focal_length': 3.04,
    'pixel_size': 0.00112,       # 更小的像元尺寸
    'optical_center_x': 640.5,   # 1280x960图像的中心
    'optical_center_y': 480.5,
    'image_width': 1280,
    'image_height': 960,
    'sensor_width': 4.8,
    'sensor_height': 3.6,
}

# 制导参数
GUIDANCE_PARAMS = {
    'N_prime': 3.5,              # 导航比 (3-5之间)
    'reference_area_at_1m': 5000, # 1米距离时的参考像素面积
    'output_limit': 10.0,        # 输出限制 (±10 m/s²)
    'min_pixel_area': 10,        # 最小有效像素面积
    'max_distance': 50.0,        # 最大估计距离 (米)
    'min_distance': 0.5,         # 最小估计距离 (米)
}

# 姿态估计参数
ATTITUDE_ESTIMATION_PARAMS = {
    'max_integration_time': 1.0,  # 最大积分时间 (秒)
    'gyro_bias_x': 0.0,          # 陀螺仪X轴零偏
    'gyro_bias_y': 0.0,          # 陀螺仪Y轴零偏
    'gyro_bias_z': 0.0,          # 陀螺仪Z轴零偏
    'gyro_noise': 0.001,         # 陀螺仪噪声 (rad/s)
    'correction_alpha': 0.1,     # 校正权重
}

# 数据记录参数
LOGGING_PARAMS = {
    'log_dir': 'logs',           # 日志目录
    'log_prefix': 'guidance_log', # 日志文件前缀
    'max_log_files': 100,        # 最大日志文件数
    'max_log_age_days': 7,       # 日志最大保留天数
    'flush_interval': 10,        # 刷新间隔 (记录条数)
}

# 飞镖物理参数
DARTS_PHYSICAL_PARAMS = {
    'mass': 0.31,                # 质量 (kg) - 两舵机镖体
    'mass_4channel': 0.365,      # 质量 (kg) - 四舵机镖体
    'max_lift_coefficient': 0.8, # 最大升力系数
    'reference_area': 0.03,      # 参考面积 (m²)
    'max_angle_of_attack': 15.0, # 最大攻角 (度)
    'max_side_slip': 10.0,       # 最大侧滑角 (度)
}

# 控制参数
CONTROL_PARAMS = {
    'max_roll_rate': 180.0,      # 最大滚转角速度 (度/秒)
    'max_pitch_rate': 90.0,      # 最大俯仰角速度 (度/秒)
    'max_yaw_rate': 60.0,        # 最大偏航角速度 (度/秒)
    'control_frequency': 100.0,  # 控制频率 (Hz)
    'servo_min_angle': -30.0,    # 舵机最小角度 (度)
    'servo_max_angle': 30.0,     # 舵机最大角度 (度)
}

# 性能限制
PERFORMANCE_LIMITS = {
    'max_g_force': 3.0,          # 最大过载 (G)
    'max_velocity': 30.0,        # 最大速度 (m/s)
    'min_velocity': 5.0,         # 最小速度 (m/s)
    'max_altitude': 20.0,        # 最大高度 (m)
    'min_altitude': 0.5,         # 最小高度 (m)
}

# 目标参数
TARGET_PARAMS = {
    'target_diameter': 0.3,      # 目标直径 (m) - 引导灯
    'target_reflectivity': 0.8,  # 目标反射率
    'min_detection_size': 5,     # 最小检测尺寸 (像素)
    'max_detection_size': 200,   # 最大检测尺寸 (像素)
    'expected_aspect_ratio': 1.0, # 期望长宽比
    'aspect_ratio_tolerance': 0.3, # 长宽比容差
}

def get_default_config():
    """
    获取默认配置
    
    返回:
        dict: 包含所有默认配置的字典
    """
    return {
        'camera': DEFAULT_CAMERA_PARAMS,
        'guidance': GUIDANCE_PARAMS,
        'attitude_estimation': ATTITUDE_ESTIMATION_PARAMS,
        'logging': LOGGING_PARAMS,
        'darts_physical': DARTS_PHYSICAL_PARAMS,
        'control': CONTROL_PARAMS,
        'performance': PERFORMANCE_LIMITS,
        'target': TARGET_PARAMS,
    }

def validate_camera_params(params):
    """
    验证相机参数
    
    参数:
        params: 相机参数字典
    
    返回:
        bool: 参数是否有效
    """
    required_keys = ['focal_length', 'pixel_size', 'optical_center_x', 
                     'optical_center_y', 'image_width', 'image_height']
    
    # 检查必需键
    for key in required_keys:
        if key not in params:
            print(f"错误：缺少相机参数 '{key}'")
            return False
    
    # 检查值范围
    if params['focal_length'] <= 0:
        print("错误：焦距必须大于0")
        return False
    
    if params['pixel_size'] <= 0:
        print("错误：像元尺寸必须大于0")
        return False
    
    if params['optical_center_x'] < 0 or params['optical_center_x'] >= params['image_width']:
        print(f"错误：光心X坐标必须在 [0, {params['image_width']}) 范围内")
        return False
    
    if params['optical_center_y'] < 0 or params['optical_center_y'] >= params['image_height']:
        print(f"错误：光心Y坐标必须在 [0, {params['image_height']}) 范围内")
        return False
    
    return True

def print_config_summary(config):
    """
    打印配置摘要
    
    参数:
        config: 配置字典
    """
    print("=" * 50)
    print("制导系统配置摘要")
    print("=" * 50)
    
    # 相机参数
    cam = config['camera']
    print(f"\n相机参数:")
    print(f"  分辨率: {cam['image_width']} × {cam['image_height']}")
    print(f"  焦距: {cam['focal_length']} mm")
    print(f"  像元尺寸: {cam['pixel_size']*1000:.2f} μm")
    print(f"  光心: ({cam['optical_center_x']:.1f}, {cam['optical_center_y']:.1f})")
    
    # 制导参数
    guide = config['guidance']
    print(f"\n制导参数:")
    print(f"  导航比 N': {guide['N_prime']}")
    print(f"  输出限制: ±{guide['output_limit']} m/s²")
    print(f"  参考面积@1m: {guide['reference_area_at_1m']} 像素")
    
    # 物理参数
    phys = config['darts_physical']
    print(f"\n物理参数:")
    print(f"  质量: {phys['mass']} kg (2通道), {phys['mass_4channel']} kg (4通道)")
    print(f"  最大攻角: {phys['max_angle_of_attack']}°")
    
    print("=" * 50)
