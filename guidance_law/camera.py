"""
相机模型模块

提供相机参数管理、像素到角度转换、距离估计等功能。
基于相机标定的光心坐标。
"""

import math


class CameraModel:
    """
    相机模型类
    
    管理相机参数，提供像素坐标到角度转换功能。
    坐标系定义：X轴向右，Y轴向下，Z轴向前（光轴方向）。
    """
    
    def __init__(self, focal_length, pixel_size, optical_center_x, optical_center_y,
                 image_width=640, image_height=480):
        """
        初始化相机模型
        
        参数:
            focal_length: 焦距 (mm)
            pixel_size: 像元尺寸 (mm/像素)
            optical_center_x: 光心X坐标 (像素)
            optical_center_y: 光心Y坐标 (像素)
            image_width: 图像宽度 (像素)
            image_height: 图像高度 (像素)
        """
        self.focal_length = focal_length
        self.pixel_size = pixel_size
        self.optical_center_x = optical_center_x
        self.optical_center_y = optical_center_y
        self.image_width = image_width
        self.image_height = image_height
        
        # 计算焦距像素单位
        self.focal_length_pixels = focal_length / pixel_size
        
        # 计算图像边界（用于验证）
        self.x_min = 0
        self.x_max = image_width - 1
        self.y_min = 0
        self.y_max = image_height - 1
    
    def pixel_to_angle(self, pixel_x, pixel_y):
        """
        将像素坐标转换为角度（相对于光心）
        
        参数:
            pixel_x: 像素X坐标
            pixel_y: 像素Y坐标
            
        返回:
            (angle_x, angle_y): X和Y方向的角度（弧度）
        """
        # 计算相对于光心的像素差
        dx = pixel_x - self.optical_center_x
        dy = pixel_y - self.optical_center_y
        
        # 计算角度（使用小角度近似）
        angle_x = math.atan2(dx, self.focal_length_pixels)
        angle_y = math.atan2(dy, self.focal_length_pixels)
        
        return angle_x, angle_y
    
    def pixel_difference_to_angle(self, pixel_dx, pixel_dy):
        """
        将像素差直接转换为角度
        
        参数:
            pixel_dx: X方向像素差
            pixel_dy: Y方向像素差
            
        返回:
            (angle_x, angle_y): X和Y方向的角度（弧度）
        """
        angle_x = math.atan2(pixel_dx, self.focal_length_pixels)
        angle_y = math.atan2(pixel_dy, self.focal_length_pixels)
        
        return angle_x, angle_y
    
    def angle_to_pixel(self, angle_x, angle_y):
        """
        将角度转换为像素坐标（相对于光心）
        
        参数:
            angle_x: X方向角度（弧度）
            angle_y: Y方向角度（弧度）
            
        返回:
            (pixel_x, pixel_y): 像素坐标
        """
        pixel_x = self.optical_center_x + self.focal_length_pixels * math.tan(angle_x)
        pixel_y = self.optical_center_y + self.focal_length_pixels * math.tan(angle_y)
        
        return pixel_x, pixel_y
    
    def validate_pixel_coordinates(self, pixel_x, pixel_y):
        """
        验证像素坐标是否在图像范围内
        
        参数:
            pixel_x: 像素X坐标
            pixel_y: 像素Y坐标
            
        返回:
            bool: 坐标是否有效
        """
        return (self.x_min <= pixel_x <= self.x_max and 
                self.y_min <= pixel_y <= self.y_max)
    
    def get_field_of_view(self):
        """
        计算相机的视场角
        
        返回:
            (fov_x, fov_y): 水平和垂直视场角（弧度）
        """
        half_width = self.image_width / 2
        half_height = self.image_height / 2
        
        fov_x = 2 * math.atan2(half_width, self.focal_length_pixels)
        fov_y = 2 * math.atan2(half_height, self.focal_length_pixels)
        
        return fov_x, fov_y
    
    def __str__(self):
        """返回相机参数的字符串表示"""
        fov_x, fov_y = self.get_field_of_view()
        fov_x_deg = math.degrees(fov_x)
        fov_y_deg = math.degrees(fov_y)
        
        return (f"CameraModel:\n"
                f"  焦距: {self.focal_length} mm\n"
                f"  像元尺寸: {self.pixel_size} mm/像素\n"
                f"  光心: ({self.optical_center_x}, {self.optical_center_y}) 像素\n"
                f"  图像尺寸: {self.image_width} × {self.image_height}\n"
                f"  视场角: {fov_x_deg:.1f}° × {fov_y_deg:.1f}°")


def estimate_distance_from_area(pixel_area, reference_area_at_1m, reference_distance=1.0):
    """
    通过像素面积估计距离
    
    原理：距离与面积的平方根成反比
    distance = reference_distance * sqrt(reference_area_at_1m / pixel_area)
    
    参数:
        pixel_area: 当前检测到的目标像素面积
        reference_area_at_1m: 在1米距离时的参考像素面积
        reference_distance: 参考距离（默认1米）
    
    返回:
        distance: 估计距离（米）
    """
    if pixel_area <= 0:
        return float('inf')
    
    # 计算距离
    distance = reference_distance * math.sqrt(reference_area_at_1m / pixel_area)
    
    return distance


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
