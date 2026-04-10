"""
数据记录器模块

提供飞行数据记录功能，支持CSV格式输出。
用于记录制导系统的输入、输出和状态信息。
"""

import csv
import os
import time
from datetime import datetime


class DataLogger:
    """
    数据记录器类
    
    记录制导系统的运行数据，支持CSV格式输出。
    自动创建时间戳命名的日志文件。
    """
    
    def __init__(self, log_dir="logs", log_prefix="guidance_log"):
        """
        初始化数据记录器
        
        参数:
            log_dir: 日志目录
            log_prefix: 日志文件前缀
        """
        self.log_dir = log_dir
        self.log_prefix = log_prefix
        self.log_file = None
        self.csv_writer = None
        self.is_recording = False
        self.record_count = 0
        
        # 确保日志目录存在
        os.makedirs(log_dir, exist_ok=True)
        
        # 字段定义
        self.fieldnames = [
            'timestamp',           # 时间戳
            'pixel_dx',           # X方向像素差
            'pixel_dy',           # Y方向像素差
            'pixel_area',         # 像素面积
            'qw', 'qx', 'qy', 'qz',  # 当前四元数
            'ref_qw', 'ref_qx', 'ref_qy', 'ref_qz',  # 基准四元数
            'angle_x', 'angle_y', # 视线角
            'q_dot_x', 'q_dot_y', # 视线角变化率
            'estimated_distance', # 估计距离
            'a_cmd_pitch',        # 俯仰通道加速度指令
            'a_cmd_yaw',          # 偏航通道加速度指令
            'dt',                 # 时间间隔
            'is_initialized',     # 是否已初始化
            'roll', 'pitch', 'yaw',  # 欧拉角
            'omega_x', 'omega_y', 'omega_z'  # 角速度
        ]
    
    def start_recording(self, filename=None):
        """
        开始记录
        
        参数:
            filename: 自定义文件名（可选）
        """
        if self.is_recording:
            print("数据记录已在运行中")
            return
        
        # 生成文件名
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.log_prefix}_{timestamp}.csv"
        
        # 完整文件路径
        filepath = os.path.join(self.log_dir, filename)
        
        try:
            # 打开文件
            self.log_file = open(filepath, 'w', newline='')
            self.csv_writer = csv.DictWriter(self.log_file, fieldnames=self.fieldnames)
            
            # 写入表头
            self.csv_writer.writeheader()
            
            self.is_recording = True
            self.record_count = 0
            
            print(f"数据记录已开始: {filepath}")
            
        except Exception as e:
            print(f"无法开始数据记录: {e}")
            self.is_recording = False
    
    def stop_recording(self):
        """停止记录"""
        if not self.is_recording:
            print("数据记录未运行")
            return
        
        try:
            if self.log_file:
                self.log_file.close()
            
            self.is_recording = False
            print(f"数据记录已停止，共记录 {self.record_count} 条数据")
            
        except Exception as e:
            print(f"停止数据记录时出错: {e}")
    
    def log(self, data):
        """
        记录一条数据
        
        参数:
            data: 包含字段数据的字典
        """
        if not self.is_recording:
            print("警告：数据记录未运行，数据未被保存")
            return
        
        try:
            # 确保数据包含所有必需字段
            record = {}
            for field in self.fieldnames:
                if field in data:
                    record[field] = data[field]
                else:
                    record[field] = None  # 填充空值
            
            # 写入CSV
            self.csv_writer.writerow(record)
            self.record_count += 1
            
            # 定期刷新缓冲区
            if self.record_count % 10 == 0:
                self.log_file.flush()
                
        except Exception as e:
            print(f"记录数据时出错: {e}")
    
    def log_guidance_data(self, timestamp, pixel_dx, pixel_dy, pixel_area,
                         current_quaternion, reference_quaternion,
                         angle_x, angle_y, q_dot_x, q_dot_y,
                         estimated_distance, a_cmd_pitch, a_cmd_yaw, dt,
                         is_initialized, euler_angles=None, omega=None):
        """
        记录制导数据（便捷方法）
        
        参数:
            timestamp: 时间戳
            pixel_dx, pixel_dy: 像素差
            pixel_area: 像素面积
            current_quaternion: 当前四元数 (qw, qx, qy, qz)
            reference_quaternion: 基准四元数 (qw, qx, qy, qz)
            angle_x, angle_y: 视线角
            q_dot_x, q_dot_y: 视线角变化率
            estimated_distance: 估计距离
            a_cmd_pitch, a_cmd_yaw: 加速度指令
            dt: 时间间隔
            is_initialized: 是否已初始化
            euler_angles: 欧拉角 (roll, pitch, yaw) 可选
            omega: 角速度 (omega_x, omega_y, omega_z) 可选
        """
        # 准备数据字典
        data = {
            'timestamp': timestamp,
            'pixel_dx': pixel_dx,
            'pixel_dy': pixel_dy,
            'pixel_area': pixel_area,
            'dt': dt,
            'is_initialized': is_initialized,
            'angle_x': angle_x,
            'angle_y': angle_y,
            'q_dot_x': q_dot_x,
            'q_dot_y': q_dot_y,
            'estimated_distance': estimated_distance,
            'a_cmd_pitch': a_cmd_pitch,
            'a_cmd_yaw': a_cmd_yaw
        }
        
        # 四元数数据
        if current_quaternion and len(current_quaternion) == 4:
            data['qw'], data['qx'], data['qy'], data['qz'] = current_quaternion
        
        if reference_quaternion and len(reference_quaternion) == 4:
            data['ref_qw'], data['ref_qx'], data['ref_qy'], data['ref_qz'] = reference_quaternion
        
        # 欧拉角数据
        if euler_angles and len(euler_angles) == 3:
            data['roll'], data['pitch'], data['yaw'] = euler_angles
        
        # 角速度数据
        if omega and len(omega) == 3:
            data['omega_x'], data['omega_y'], data['omega_z'] = omega
        
        # 记录数据
        self.log(data)
    
    def get_log_files(self):
        """
        获取日志目录中的所有日志文件
        
        返回:
            list: 日志文件列表
        """
        if not os.path.exists(self.log_dir):
            return []
        
        log_files = []
        for file in os.listdir(self.log_dir):
            if file.endswith('.csv') and file.startswith(self.log_prefix):
                log_files.append(os.path.join(self.log_dir, file))
        
        return sorted(log_files)
    
    def get_latest_log_file(self):
        """
        获取最新的日志文件
        
        返回:
            str: 最新日志文件路径，如果没有则返回None
        """
        log_files = self.get_log_files()
        if log_files:
            return log_files[-1]
        return None
    
    def clear_old_logs(self, max_age_days=7):
        """
        清理旧的日志文件
        
        参数:
            max_age_days: 最大保留天数
        """
        if not os.path.exists(self.log_dir):
            return
        
        current_time = time.time()
        cutoff_time = current_time - (max_age_days * 24 * 3600)
        
        deleted_count = 0
        for file in os.listdir(self.log_dir):
            if file.endswith('.csv') and file.startswith(self.log_prefix):
                filepath = os.path.join(self.log_dir, file)
                file_mtime = os.path.getmtime(filepath)
                
                if file_mtime < cutoff_time:
                    try:
                        os.remove(filepath)
                        deleted_count += 1
                        print(f"已删除旧日志文件: {file}")
                    except Exception as e:
                        print(f"删除文件 {file} 时出错: {e}")
        
        if deleted_count > 0:
            print(f"共删除 {deleted_count} 个旧日志文件")
    
    def __del__(self):
        """析构函数，确保文件被关闭"""
        if self.is_recording and self.log_file:
            try:
                self.log_file.close()
            except:
                pass
