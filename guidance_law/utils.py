"""
工具函数模块

提供各种辅助函数，包括数值处理、限制函数、单位转换等。
"""

import math
import numpy as np


def limit_value(value, min_val, max_val):
    """
    限制值在指定范围内
    
    参数:
        value: 输入值
        min_val: 最小值
        max_val: 最大值
    
    返回:
        限制后的值
    """
    if value < min_val:
        return min_val
    elif value > max_val:
        return max_val
    else:
        return value


def normalize_vector(v):
    """
    归一化向量
    
    参数:
        v: 输入向量 [x, y, z]
    
    返回:
        归一化后的向量
    """
    v = np.array(v)
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def calculate_pixel_difference(target_x, target_y, optical_center_x, optical_center_y):
    """
    计算目标位置相对于光心的像素差
    
    参数:
        target_x: 目标X坐标（像素）
        target_y: 目标Y坐标（像素）
        optical_center_x: 光心X坐标（像素）
        optical_center_y: 光心Y坐标（像素）
    
    返回:
        (pixel_dx, pixel_dy): 像素差
    """
    pixel_dx = target_x - optical_center_x
    pixel_dy = target_y - optical_center_y
    
    return pixel_dx, pixel_dy


def radians_to_degrees(rad):
    """
    弧度转角度
    
    参数:
        rad: 弧度值
    
    返回:
        角度值
    """
    return rad * 180.0 / math.pi


def degrees_to_radians(deg):
    """
    角度转弧度
    
    参数:
        deg: 角度值
    
    返回:
        弧度值
    """
    return deg * math.pi / 180.0


def low_pass_filter(current_value, previous_value, alpha):
    """
    一阶低通滤波器
    
    参数:
        current_value: 当前值
        previous_value: 前一个值
        alpha: 滤波系数 (0-1)，越小滤波越强
    
    返回:
        滤波后的值
    """
    return alpha * current_value + (1 - alpha) * previous_value


def moving_average(values, new_value, window_size=5):
    """
    移动平均滤波器
    
    参数:
        values: 历史值列表
        new_value: 新值
        window_size: 窗口大小
    
    返回:
        移动平均值
    """
    values.append(new_value)
    if len(values) > window_size:
        values.pop(0)
    
    return sum(values) / len(values)


def median_filter(values, new_value, window_size=5):
    """
    中值滤波器
    
    参数:
        values: 历史值列表
        new_value: 新值
        window_size: 窗口大小
    
    返回:
        中值
    """
    values.append(new_value)
    if len(values) > window_size:
        values.pop(0)
    
    sorted_values = sorted(values)
    return sorted_values[len(sorted_values) // 2]


def calculate_angle_between_vectors(v1, v2):
    """
    计算两个向量之间的夹角
    
    参数:
        v1: 第一个向量 [x, y, z]
        v2: 第二个向量 [x, y, z]
    
    返回:
        夹角（弧度）
    """
    v1 = np.array(v1)
    v2 = np.array(v2)
    
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    
    if norm_v1 == 0 or norm_v2 == 0:
        return 0.0
    
    cos_angle = dot_product / (norm_v1 * norm_v2)
    cos_angle = limit_value(cos_angle, -1.0, 1.0)
    
    return math.acos(cos_angle)


def rotate_vector_2d(v, angle):
    """
    二维向量旋转
    
    参数:
        v: 输入向量 [x, y]
        angle: 旋转角度（弧度）
    
    返回:
        旋转后的向量 [x', y']
    """
    x, y = v
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    
    x_rotated = x * cos_a - y * sin_a
    y_rotated = x * sin_a + y * cos_a
    
    return [x_rotated, y_rotated]


def calculate_distance(point1, point2):
    """
    计算两点之间的欧氏距离
    
    参数:
        point1: 第一个点 [x, y, z]
        point2: 第二个点 [x, y, z]
    
    返回:
        距离
    """
    p1 = np.array(point1)
    p2 = np.array(point2)
    
    return np.linalg.norm(p1 - p2)


def interpolate(value, from_min, from_max, to_min, to_max):
    """
    线性插值
    
    参数:
        value: 输入值
        from_min, from_max: 输入范围
        to_min, to_max: 输出范围
    
    返回:
        插值后的值
    """
    # 限制输入值在范围内
    value = limit_value(value, from_min, from_max)
    
    # 线性插值
    from_range = from_max - from_min
    to_range = to_max - to_min
    
    if from_range == 0:
        return to_min
    
    normalized = (value - from_min) / from_range
    return to_min + normalized * to_range


def wrap_angle_180(angle_deg):
    """
    将角度包装到[-180, 180]度范围
    
    参数:
        angle_deg: 输入角度（度）
    
    返回:
        包装后的角度（度）
    """
    angle_deg = angle_deg % 360
    if angle_deg > 180:
        angle_deg -= 360
    elif angle_deg < -180:
        angle_deg += 360
    return angle_deg


def wrap_angle_360(angle_deg):
    """
    将角度包装到[0, 360]度范围
    
    参数:
        angle_deg: 输入角度（度）
    
    返回:
        包装后的角度（度）
    """
    angle_deg = angle_deg % 360
    if angle_deg < 0:
        angle_deg += 360
    return angle_deg


def calculate_derivative(current_value, previous_value, dt):
    """
    计算导数
    
    参数:
        current_value: 当前值
        previous_value: 前一个值
        dt: 时间间隔
    
    返回:
        导数值
    """
    if dt <= 0:
        return 0.0
    
    return (current_value - previous_value) / dt


def calculate_integral(values, dt):
    """
    计算积分（梯形法则）
    
    参数:
        values: 值列表
        dt: 时间间隔
    
    返回:
        积分值
    """
    if len(values) < 2:
        return 0.0
    
    integral = 0.0
    for i in range(1, len(values)):
        integral += (values[i] + values[i-1]) * dt / 2
    
    return integral


def is_within_tolerance(value, target, tolerance):
    """
    检查值是否在目标值的容差范围内
    
    参数:
        value: 当前值
        target: 目标值
        tolerance: 容差
    
    返回:
        bool: 是否在容差范围内
    """
    return abs(value - target) <= tolerance


def calculate_rms(values):
    """
    计算均方根值
    
    参数:
        values: 值列表
    
    返回:
        均方根值
    """
    if not values:
        return 0.0
    
    squares = [v * v for v in values]
    mean_square = sum(squares) / len(squares)
    
    return math.sqrt(mean_square)


def format_time(seconds):
    """
    格式化时间显示
    
    参数:
        seconds: 秒数
    
    返回:
        格式化后的时间字符串
    """
    if seconds < 60:
        return f"{seconds:.2f}s"
    elif seconds < 3600:
        minutes = int(seconds // 60)
        secs = seconds % 60
        return f"{minutes}m {secs:.1f}s"
    else:
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = seconds % 60
        return f"{hours}h {minutes}m {secs:.1f}s"
