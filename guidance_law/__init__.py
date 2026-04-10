"""
制导率库 - Guidance Law Library

基于广义比例导引律的制导算法库，专为飞镖制导系统设计。
支持基于发射前四元数的基准平面计算、欧拉角速度积分、像素面积距离估计等功能。

主要模块：
- core: 制导核心算法
- coordinate: 坐标变换和四元数操作
- camera: 相机模型和距离估计
- estimator: 姿态估计器（欧拉角速度积分）
- logger: 数据记录器
- config: 配置参数
- utils: 工具函数

版本: 1.0.0
作者: 基于飞镖技术报告实现
"""

from .core import GuidanceSystem
from .coordinate import (
    quaternion_to_euler_zyx,
    euler_to_quaternion_zyx,
    quaternion_multiply,
    quaternion_conjugate,
    quaternion_rotate,
    integrate_euler_angles,
    normalize_angle
)
from .camera import CameraModel, estimate_distance_from_area
from .estimator import AttitudeEstimator
from .logger import DataLogger
from .config import DEFAULT_CAMERA_PARAMS, get_default_config
from .utils import limit_value, calculate_pixel_difference, radians_to_degrees, degrees_to_radians

__version__ = "1.0.0"
__all__ = [
    'GuidanceSystem',
    'quaternion_to_euler_zyx',
    'euler_to_quaternion_zyx',
    'quaternion_multiply',
    'quaternion_conjugate',
    'quaternion_rotate',
    'integrate_euler_angles',
    'normalize_angle',
    'CameraModel',
    'estimate_distance_from_area',
    'AttitudeEstimator',
    'DataLogger',
    'DEFAULT_CAMERA_PARAMS',
    'get_default_config',
    'limit_value',
    'calculate_pixel_difference',
    'radians_to_degrees',
    'degrees_to_radians'
]
