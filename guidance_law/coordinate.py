"""
坐标变换模块

提供四元数操作、欧拉角转换、坐标系变换等功能。
基于Z(yaw)→Y(pitch)→X(roll)的欧拉角顺序。
"""

import math
import numpy as np


def quaternion_to_euler_zyx(qw, qx, qy, qz):
    """
    四元数转欧拉角 (Z-Y-X顺序: yaw -> pitch -> roll)
    
    参数:
        qw, qx, qy, qz: 四元数分量 (qw为实部)
    
    返回:
        (roll, pitch, yaw): 欧拉角，单位: 弧度
    """
    # 计算滚转角 (phi, X轴)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # 计算俯仰角 (theta, Y轴)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        # 使用90度处理奇异点
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # 计算偏航角 (psi, Z轴)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def euler_to_quaternion_zyx(roll, pitch, yaw):
    """
    欧拉角转四元数 (Z-Y-X顺序: yaw -> pitch -> roll)
    
    参数:
        roll, pitch, yaw: 欧拉角，单位: 弧度
    
    返回:
        (qw, qx, qy, qz): 四元数分量
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return qw, qx, qy, qz


def quaternion_multiply(q1, q2):
    """
    四元数乘法: q = q1 ⊗ q2
    
    参数:
        q1, q2: 四元数 (qw, qx, qy, qz)
    
    返回:
        乘积四元数 (qw, qx, qy, qz)
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return w, x, y, z


def quaternion_conjugate(q):
    """
    四元数共轭
    
    参数:
        q: 四元数 (qw, qx, qy, qz)
    
    返回:
        共轭四元数 (qw, -qx, -qy, -qz)
    """
    w, x, y, z = q
    return w, -x, -y, -z


def quaternion_rotate(q, v):
    """
    使用四元数旋转向量
    
    参数:
        q: 旋转四元数 (qw, qx, qy, qz)
        v: 三维向量 [x, y, z]
    
    返回:
        旋转后的向量 [x', y', z']
    """
    # 将向量转换为纯四元数
    v_q = (0.0, v[0], v[1], v[2])
    
    # 计算旋转: v' = q ⊗ v ⊗ q*
    q_conj = quaternion_conjugate(q)
    v_rotated = quaternion_multiply(quaternion_multiply(q, v_q), q_conj)
    
    # 返回向量部分
    return [v_rotated[1], v_rotated[2], v_rotated[3]]


def integrate_euler_angles(roll, pitch, yaw, omega_x, omega_y, omega_z, dt):
    """
    欧拉角速度积分（Z-Y-X顺序）
    
    根据技术报告中的公式：
    roll_dot = omega_x * cos(yaw) + omega_z * sin(yaw)
    pitch_dot = omega_y * cos(roll) - omega_z * sin(roll) * cos(yaw) + omega_x * sin(roll) * sin(yaw)
    yaw_dot = (omega_z * cos(roll) * cos(yaw) - omega_x * cos(roll) * sin(yaw)) / cos(pitch)
    
    参数:
        roll, pitch, yaw: 当前欧拉角（弧度）
        omega_x, omega_y, omega_z: 机体角速度（弧度/秒）
        dt: 时间间隔（秒）
    
    返回:
        更新后的欧拉角（弧度）
    """
    # 避免除零
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) < 1e-6:
        cos_pitch = 1e-6 if cos_pitch >= 0 else -1e-6
    
    # 计算欧拉角变化率
    roll_dot = omega_x * math.cos(yaw) + omega_z * math.sin(yaw)
    pitch_dot = omega_y * math.cos(roll) - omega_z * math.sin(roll) * math.cos(yaw) + omega_x * math.sin(roll) * math.sin(yaw)
    yaw_dot = (omega_z * math.cos(roll) * math.cos(yaw) - omega_x * math.cos(roll) * math.sin(yaw)) / cos_pitch
    
    # 积分更新
    roll_new = roll + roll_dot * dt
    pitch_new = pitch + pitch_dot * dt
    yaw_new = yaw + yaw_dot * dt
    
    # 角度归一化
    roll_new = normalize_angle(roll_new)
    pitch_new = normalize_angle(pitch_new)
    yaw_new = normalize_angle(yaw_new)
    
    return roll_new, pitch_new, yaw_new


def normalize_angle(angle):
    """
    将角度归一化到[-π, π]范围
    
    参数:
        angle: 输入角度（弧度）
    
    返回:
        归一化后的角度（弧度）
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def compute_relative_quaternion(current_q, reference_q):
    """
    计算当前四元数相对于基准四元数的相对旋转
    
    相对四元数: q_rel = current_q ⊗ conj(reference_q)
    
    参数:
        current_q: 当前四元数 (qw, qx, qy, qz)
        reference_q: 基准四元数 (qw, qx, qy, qz)
    
    返回:
        相对四元数 (qw, qx, qy, qz)
    """
    # 计算基准四元数的共轭
    ref_conj = quaternion_conjugate(reference_q)
    
    # 计算相对四元数
    q_rel = quaternion_multiply(current_q, ref_conj)
    
    return q_rel
