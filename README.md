# 制导率库 (Guidance Law Library)

基于广义比例导引律的飞镖制导算法库，专为RoboMaster飞镖制导系统设计。

## 概述

本库根据飞镖技术报告中的制导方案实现，提供完整的制导算法解决方案，包括：
- 基于发射前四元数的基准平面确定
- 像素差到视线角转换（考虑相机光心）
- 距离估计（基于像素面积）
- 广义比例导引律计算
- 欧拉角速度积分姿态估计
- 完整的飞行数据记录

## 特性

### 1. 核心算法
- **广义比例导引律**：基于技术报告中的最优导引规律
- **基准平面确定**：使用发射前四元数建立参考坐标系
- **视线角计算**：考虑相机光心的精确角度转换
- **距离估计**：基于像素面积的实时距离估计

### 2. 姿态估计
- **四元数操作**：完整的四元数运算功能
- **欧拉角速度积分**：在IMU失效时进行短时间姿态估计
- **姿态平滑**：历史数据平滑和校正

### 3. 数据记录
- **CSV格式日志**：完整的飞行数据记录
- **自动文件管理**：时间戳命名，自动清理旧日志
- **丰富字段**：包含所有制导相关参数

### 4. 配置系统
- **默认参数**：基于常见飞镖系统的默认配置
- **参数验证**：自动验证配置参数有效性
- **灵活定制**：支持自定义相机、制导、控制参数

## 安装

### 依赖
- Python 3.7+
- NumPy
- 可选：matplotlib（用于数据分析）

### 安装步骤
```bash
# 克隆或下载库文件
git clone <repository-url>

# 安装依赖
pip install numpy

# 可选：安装数据分析工具
pip install matplotlib pandas
```

## 快速开始

### 基本使用
```python
from guidance_law import GuidanceSystem, DEFAULT_CAMERA_PARAMS

# 1. 创建制导系统
guidance = GuidanceSystem(
    camera_params=DEFAULT_CAMERA_PARAMS,
    N_prime=3.5,              # 导航比
    reference_area_at_1m=5000, # 1米距离的参考像素面积
    output_limit=10.0,        # 输出限制
    log_dir="flight_logs",    # 日志目录
    log_prefix="flight"       # 日志文件前缀
)

# 2. 设置发射前基准四元数
launch_quaternion = (1.0, 0.0, 0.0, 0.0)  # 无旋转
guidance.set_reference_quaternion(*launch_quaternion)

# 3. 在主循环中更新制导系统
while flying:
    # 从视觉系统获取目标信息
    target_x = 350  # 像素坐标
    target_y = 250
    pixel_area = 800
    
    # 计算像素差
    pixel_dx = target_x - DEFAULT_CAMERA_PARAMS['optical_center_x']
    pixel_dy = target_y - DEFAULT_CAMERA_PARAMS['optical_center_y']
    
    # 从IMU获取当前姿态
    current_quaternion = (0.9986, 0.0, 0.0523, 0.0)  # 绕Y轴旋转3度
    
    # 更新制导系统，获取控制指令
    a_cmd_pitch, a_cmd_yaw = guidance.update(
        pixel_dx=pixel_dx,
        pixel_dy=pixel_dy,
        pixel_area=pixel_area,
        current_quaternion=current_quaternion,
        is_launch_detected=False
    )
    
    print(f"制导指令: 俯仰={a_cmd_pitch:.2f} m/s², 偏航={a_cmd_yaw:.2f} m/s²")

# 4. 飞行结束后停止系统
guidance.stop()
```

### 使用陀螺仪积分
```python
# 当IMU失效或需要高频更新时，使用陀螺仪积分
omega = (0.05, 0.02, 0.01)  # 角速度 (rad/s)

a_cmd_pitch, a_cmd_yaw = guidance.update(
    pixel_dx=pixel_dx,
    pixel_dy=pixel_dy,
    pixel_area=pixel_area,
    omega=omega,  # 使用角速度而不是四元数
    is_launch_detected=False
)
```

## 库结构

```
guidance_law/
├── __init__.py          # 主模块
├── core.py              # 制导核心算法
├── coordinate.py        # 坐标变换和四元数操作
├── camera.py           # 相机模型和距离估计
├── estimator.py        # 姿态估计器（欧拉角速度积分）
├── logger.py           # 数据记录器
├── config.py           # 配置参数
└── utils.py            # 工具函数

examples/
├── test_guidance.py    # 测试用例
└── example_usage.py    # 使用示例
```

## 详细说明

### 1. 制导算法原理

#### 广义比例导引律
```
a_cmd = N' × R × q_dot
```
其中：
- `a_cmd`：法向加速度指令 (m/s²)
- `N'`：导航比 (通常3-5)
- `R`：弹目距离 (m)
- `q_dot`：视线角变化率 (rad/s)

#### 基准平面确定
- 在发射前记录飞镖姿态四元数作为基准
- 所有后续计算都相对于这个基准平面
- 确保制导指令与发射方向一致

#### 距离估计
```
distance = reference_distance × sqrt(reference_area_at_1m / pixel_area)
```
基于目标像素面积与距离的平方反比关系。

### 2. 姿态估计

#### 欧拉角速度积分
在飞行过程中，当IMU在失重状态下失效时，使用角速度积分进行短时间姿态估计：
```
roll_dot = ω_x × cos(yaw) + ω_z × sin(yaw)
pitch_dot = ω_y × cos(roll) - ω_z × sin(roll) × cos(yaw) + ω_x × sin(roll) × sin(yaw)
yaw_dot = (ω_z × cos(roll) × cos(yaw) - ω_x × cos(roll) × sin(yaw)) / cos(pitch)
```

### 3. 数据记录

#### 记录字段
- 时间戳、像素差、像素面积
- 当前四元数、基准四元数
- 视线角、视线角变化率
- 估计距离、制导指令
- 欧拉角、角速度
- 系统状态标志

#### 日志文件
- 自动创建时间戳命名的CSV文件
- 支持定期清理旧日志
- 便于后期分析和调试

## 配置参数

### 相机参数
```python
DEFAULT_CAMERA_PARAMS = {
    'focal_length': 3.04,        # 焦距 (mm)
    'pixel_size': 0.0014,        # 像元尺寸 (mm/像素)
    'optical_center_x': 320.5,   # 光心X坐标
    'optical_center_y': 240.5,   # 光心Y坐标
    'image_width': 640,          # 图像宽度
    'image_height': 480,         # 图像高度
}
```

### 制导参数
```python
GUIDANCE_PARAMS = {
    'N_prime': 3.5,              # 导航比
    'reference_area_at_1m': 5000, # 参考像素面积
    'output_limit': 10.0,        # 输出限制
    'min_pixel_area': 10,        # 最小有效像素面积
    'max_distance': 50.0,        # 最大估计距离
    'min_distance': 0.5,         # 最小估计距离
}
```

### 物理参数
```python
DARTS_PHYSICAL_PARAMS = {
    'mass': 0.31,                # 质量 (kg) - 两舵机镖体
    'mass_4channel': 0.365,      # 质量 (kg) - 四舵机镖体
    'max_lift_coefficient': 0.8, # 最大升力系数
    'reference_area': 0.03,      # 参考面积 (m²)
}
```

## 测试和验证

### 运行测试
```bash
python test_guidance.py
```

### 运行示例
```bash
python example_usage.py
```

### 测试内容
1. 坐标变换函数测试
2. 相机模型测试
3. 姿态估计器测试
4. 数据记录器测试
5. 制导系统集成测试
6. 工具函数测试

## 性能优化

### 针对Kendryte K230芯片
- 避免使用复杂的矩阵运算
- 使用查表法替代实时三角函数计算
- 优化内存使用，减少动态分配
- 使用定点数运算提高速度

### 实时性考虑
- 单次更新处理时间 < 1ms
- 支持100Hz以上的更新频率
- 内存占用 < 64KB

## 使用建议

### 1. 相机标定
- 在实际使用前必须进行相机标定
- 准确测量焦距和像元尺寸
- 确定光心坐标

### 2. 参数调优
- 根据飞镖性能调整输出限制
- 根据目标特性调整参考像素面积
- 根据飞行环境调整导航比

### 3. 数据记录
- 每次飞行都记录数据
- 定期分析日志文件优化参数
- 使用数据验证算法效果

### 4. 故障处理
- IMU失效时自动切换到陀螺仪积分
- 无效输入时返回安全值
- 记录所有异常情况

## 故障排除

### 常见问题

#### 1. 制导指令过大
- 检查相机参数是否正确
- 验证像素差计算
- 调整输出限制参数

#### 2. 姿态估计漂移
- 检查陀螺仪零偏
- 减少积分时间
- 增加四元数校正频率

#### 3. 距离估计不准确
- 重新标定参考像素面积
- 检查目标检测算法
- 调整距离估计参数

#### 4. 性能问题
- 检查处理时间是否过长
- 优化算法复杂度
- 考虑使用硬件加速

## 开发指南

### 添加新功能
1. 在相应模块中添加新函数或类
2. 更新`__init__.py`中的导出列表
3. 添加测试用例
4. 更新文档

### 代码规范
- 使用有意义的变量名和函数名
- 添加详细的文档字符串
- 遵循PEP 8编码规范
- 添加类型提示

### 版本管理
- 使用语义化版本号
- 记录所有API变更
- 维护变更日志

## 许可证

本项目基于MIT许可证开源。

## 致谢

- 基于RoboMaster飞镖技术报告实现
- 参考了导弹制导相关理论
- 感谢所有贡献者和测试者

## 联系方式

如有问题或建议，请通过以下方式联系：
- 提交GitHub Issue
- 发送邮件至开发者

---

**版本**: 1.0.0  
**最后更新**: 2025年4月  
**作者**: 基于飞镖技术报告实现
