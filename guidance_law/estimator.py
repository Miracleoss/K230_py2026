"""
姿态估计器模块

提供基于欧拉角速度积分的短时间姿态更新功能。
用于在IMU失效或需要高频更新时进行姿态估计。
"""

import time
import math
from .coordinate import (
    quaternion_to_euler_zyx,
    euler_to_quaternion_zyx,
    integrate_euler_angles,
    normalize_angle
)


class AttitudeEstimator:
    """
    姿态估计器类
    
    使用欧拉角速度积分进行短时间姿态更新。
    支持两种模式：
    1. 从四元数初始化/校正
    2. 从陀螺仪角速度积分更新
    """
    
    def __init__(self, initial_roll=0.0, initial_pitch=0.0, initial_yaw=0.0):
        """
        初始化姿态估计器
        
        参数:
            initial_roll: 初始滚转角（弧度）
            initial_pitch: 初始俯仰角（弧度）
            initial_yaw: 初始偏航角（弧度）
        """
        self.roll = initial_roll
        self.pitch = initial_pitch
        self.yaw = initial_yaw
        
        self.last_time = None
        self.is_initialized = False
        self.update_count = 0
        
        # 角速度历史（用于平滑）
        self.omega_history = []
        self.max_history_size = 5
        
        # 积分误差统计
        self.integration_error = 0.0
        self.max_integration_time = 1.0  # 最大积分时间（秒）
    
    def initialize(self, roll, pitch, yaw):
        """
        初始化姿态
        
        参数:
            roll: 滚转角（弧度）
            pitch: 俯仰角（弧度）
            yaw: 偏航角（弧度）
        """
        self.roll = normalize_angle(roll)
        self.pitch = normalize_angle(pitch)
        self.yaw = normalize_angle(yaw)
        
        self.is_initialized = True
        self.last_time = time.time()
        self.update_count = 0
        self.omega_history = []
        
        print(f"姿态估计器已初始化: roll={math.degrees(roll):.1f}°, "
              f"pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°")
    
    def initialize_from_quaternion(self, qw, qx, qy, qz):
        """
        从四元数初始化姿态
        
        参数:
            qw, qx, qy, qz: 四元数分量
        """
        roll, pitch, yaw = quaternion_to_euler_zyx(qw, qx, qy, qz)
        self.initialize(roll, pitch, yaw)
    
    def update_from_quaternion(self, qw, qx, qy, qz):
        """
        从四元数更新姿态（用于校正）
        
        参数:
            qw, qx, qy, qz: 四元数分量
        """
        roll, pitch, yaw = quaternion_to_euler_zyx(qw, qx, qy, qz)
        
        # 如果尚未初始化，直接设置
        if not self.is_initialized:
            self.initialize(roll, pitch, yaw)
        else:
            # 已初始化，进行平滑更新（混合）
            alpha = 0.1  # 校正权重
            self.roll = (1 - alpha) * self.roll + alpha * normalize_angle(roll)
            self.pitch = (1 - alpha) * self.pitch + alpha * normalize_angle(pitch)
            self.yaw = (1 - alpha) * self.yaw + alpha * normalize_angle(yaw)
        
        self.last_time = time.time()
        self.update_count += 1
    
    def update_from_gyro(self, omega_x, omega_y, omega_z):
        """
        从陀螺仪角速度积分更新姿态
        
        参数:
            omega_x: X轴角速度（弧度/秒）
            omega_y: Y轴角速度（弧度/秒）
            omega_z: Z轴角速度（弧度/秒）
        
        返回:
            (roll, pitch, yaw): 更新后的欧拉角（弧度）
        """
        if not self.is_initialized:
            raise ValueError("姿态估计器未初始化，请先调用initialize或initialize_from_quaternion")
        
        current_time = time.time()
        
        # 计算时间间隔
        if self.last_time is None:
            dt = 0.01  # 默认10ms
        else:
            dt = current_time - self.last_time
            # 限制最大时间间隔
            if dt > 0.1:  # 100ms
                dt = 0.1
                print(f"警告：时间间隔过大 ({dt:.3f}s)，已限制为0.1s")
        
        # 存储角速度历史（用于平滑）
        self.omega_history.append((omega_x, omega_y, omega_z))
        if len(self.omega_history) > self.max_history_size:
            self.omega_history.pop(0)
        
        # 使用平均角速度（可选）
        if len(self.omega_history) > 1:
            avg_omega_x = sum(w[0] for w in self.omega_history) / len(self.omega_history)
            avg_omega_y = sum(w[1] for w in self.omega_history) / len(self.omega_history)
            avg_omega_z = sum(w[2] for w in self.omega_history) / len(self.omega_history)
        else:
            avg_omega_x, avg_omega_y, avg_omega_z = omega_x, omega_y, omega_z
        
        # 欧拉角速度积分
        self.roll, self.pitch, self.yaw = integrate_euler_angles(
            self.roll, self.pitch, self.yaw,
            avg_omega_x, avg_omega_y, avg_omega_z,
            dt
        )
        
        # 更新积分误差（简单估计）
        self.integration_error += dt * 0.01  # 假设0.01 rad/s的误差率
        
        # 检查积分时间是否过长
        total_integration_time = self.update_count * dt
        if total_integration_time > self.max_integration_time:
            print(f"警告：积分时间过长 ({total_integration_time:.1f}s)，建议使用四元数校正")
        
        self.last_time = current_time
        self.update_count += 1
        
        return self.roll, self.pitch, self.yaw
    
    def get_euler_angles(self):
        """
        获取当前欧拉角
        
        返回:
            (roll, pitch, yaw): 欧拉角（弧度）
        """
        return self.roll, self.pitch, self.yaw
    
    def get_euler_angles_degrees(self):
        """
        获取当前欧拉角（度）
        
        返回:
            (roll, pitch, yaw): 欧拉角（度）
        """
        roll_deg = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)
        yaw_deg = math.degrees(self.yaw)
        
        return roll_deg, pitch_deg, yaw_deg
    
    def get_quaternion(self):
        """
        获取当前姿态的四元数表示
        
        返回:
            (qw, qx, qy, qz): 四元数分量
        """
        return euler_to_quaternion_zyx(self.roll, self.pitch, self.yaw)
    
    def get_status(self):
        """
        获取估计器状态
        
        返回:
            dict: 包含估计器状态信息的字典
        """
        roll_deg, pitch_deg, yaw_deg = self.get_euler_angles_degrees()
        
        return {
            'initialized': self.is_initialized,
            'update_count': self.update_count,
            'roll_deg': roll_deg,
            'pitch_deg': pitch_deg,
            'yaw_deg': yaw_deg,
            'integration_error': self.integration_error,
            'omega_history_size': len(self.omega_history)
        }
    
    def reset(self):
        """重置估计器"""
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = None
        self.is_initialized = False
        self.update_count = 0
        self.omega_history = []
        self.integration_error = 0.0
        
        print("姿态估计器已重置")
