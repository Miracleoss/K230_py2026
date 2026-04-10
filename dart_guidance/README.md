# 飞镖制导库 (Dart Guidance Library)

基于广义比例导引法的C语言下位机库，用于飞镖制导控制。根据技术报告中的控制率实现，输入像素差和四元数，输出解耦的控制参数（加速度指令）。

## 功能特性

- **广义比例导引法**：实现 `n = N' × R × q_dot` 算法
- **距离估计**：基于像素面积进行距离估计
- **姿态处理**：使用四元数进行姿态估计和坐标转换
- **解耦输出**：输出俯仰和偏航方向的加速度指令
- **实时性**：适合嵌入式系统，算法复杂度低
- **可配置**：所有参数可通过配置结构体调整

## 算法原理

### 输入
1. **视觉输入**：目标与光心的像素差 (dx, dy) 和像素面积
2. **姿态输入**：当前四元数（相对于发射基准面）
3. **基准四元数**：发射瞬间的姿态（用于相对姿态计算）

### 处理流程
1. **距离估计**：`R = f(像素面积)`，面积越大距离越近
2. **视线角计算**：`θ = atan2(dy, dx) - θ_velocity`
3. **视线角速度**：`q_dot = (θ_current - θ_previous) / Δt`
4. **比例导引**：`加速度 = N' × R × q_dot`
5. **解耦输出**：分配为俯仰和偏航加速度

### 输出
- `accel_pitch`：俯仰方向加速度 (m/s²)
- `accel_yaw`：偏航方向加速度 (m/s²)
- `los_rate`：视线角速度 (rad/s)
- `est_distance`：估计距离 (m)

## 项目结构

```
dart_guidance/
├── include/                    # 头文件
│   ├── dart_guidance.h        # 主头文件
│   ├── guidance_types.h       # 数据类型定义
│   ├── guidance_math.h        # 数学工具函数
│   └── guidance_core.h        # 核心算法头文件
├── src/                       # 源文件
│   ├── dart_guidance.c        # 主接口实现
│   ├── guidance_math.c        # 数学函数实现
│   └── guidance_core.c        # 核心算法实现
├── examples/                  # 示例程序
│   └── simple_example.c       # 使用示例
├── test/                      # 测试代码
│   └── test_basic.c           # 基本功能测试
├── Makefile                   # 编译配置
└── README.md                  # 本文档
```

## 快速开始

### 编译库

```bash
cd /workspace/dart_guidance
make all
```

### 运行示例

```bash
make run-example
```

### 运行测试

```bash
make run-test
```

## API使用示例

### 基本使用流程

```c
#include "dart_guidance.h"

int main() {
    // 1. 配置参数
    GuidanceConfig config = {
        .nav_ratio = 4.0f,
        .min_distance = 5.0f,
        .max_distance = 25.0f,
        .min_area = 10.0f,
        .max_area = 1000.0f,
        .focal_length = 500.0f,
        .velocity_angle = 0.1f,
        .filter_alpha = 0.3f
    };
    
    // 2. 初始化
    guidance_init(config, 1.0f, 0.0f, 0.0f, 0.0f);
    
    // 3. 设置发射基准四元数
    guidance_set_reference_quaternion(qw, qx, qy, qz);
    
    // 4. 主循环
    while (1) {
        // 更新视觉数据
        guidance_update_vision(dx, dy, area, timestamp);
        
        // 更新姿态数据
        guidance_update_attitude(qw, qx, qy, qz, timestamp);
        
        // 计算制导指令
        GuidanceOutput output = guidance_calculate();
        
        if (output.valid) {
            // 使用输出
            control_pitch(output.accel_pitch);
            control_yaw(output.accel_yaw);
        }
    }
    
    // 5. 重置（准备下一次发射）
    guidance_reset();
    
    return 0;
}
```

### 与Python视觉模块集成

```python
# Python端伪代码
import ctypes

# 加载C库
guidance_lib = ctypes.CDLL('./libdart_guidance.so')

# 初始化
config = GuidanceConfig(...)
guidance_lib.guidance_init(config, ...)

# 在主循环中
while True:
    # 从视觉模块获取数据
    blob = find_target_blob()
    if blob:
        # 计算像素差
        dx = blob.cx - image_center_x
        dy = blob.cy - image_center_y
        
        # 调用C库
        guidance_lib.guidance_update_vision(dx, dy, blob.area, timestamp)
        guidance_lib.guidance_update_attitude(qw, qx, qy, qz, timestamp)
        
        # 获取制导指令
        output = guidance_lib.guidance_calculate()
        
        if output.valid:
            control_pitch(output.accel_pitch)
            control_yaw(output.accel_yaw)
```

## API参考

### 主要函数

| 函数 | 描述 |
|------|------|
| `guidance_init()` | 初始化制导系统 |
| `guidance_set_reference_quaternion()` | 设置发射基准四元数 |
| `guidance_update_vision()` | 更新视觉数据 |
| `guidance_update_attitude()` | 更新姿态数据 |
| `guidance_calculate()` | 计算制导指令 |
| `guidance_set_distance()` | 设置外部距离估计 |
| `guidance_set_velocity_angle()` | 设置速度夹角 |
| `guidance_reset()` | 重置系统状态 |

### 数据结构

#### GuidanceConfig
```c
typedef struct {
    float nav_ratio;      // 导航比 N' (3-5)
    float min_distance;   // 最小距离 (m)
    float max_distance;   // 最大距离 (m)
    float min_area;       // 最小像素面积
    float max_area;       // 最大像素面积
    float focal_length;   // 焦距 (像素)
    float velocity_angle; // 速度与基准面夹角 (rad)
    float filter_alpha;   // 低通滤波器系数 (0-1)
} GuidanceConfig;
```

#### GuidanceOutput
```c
typedef struct {
    float accel_pitch;   // 俯仰方向加速度 (m/s²)
    float accel_yaw;     // 偏航方向加速度 (m/s²)
    float los_rate;      // 视线角速度 (rad/s)
    float est_distance;  // 估计距离 (m)
    bool valid;          // 输出是否有效
} GuidanceOutput;
```

## 参数调优

### 导航比 (nav_ratio)
- **范围**：3.0 - 5.0
- **建议值**：4.0
- **作用**：控制制导系统的响应速度

### 距离估计参数
- `min_distance`/`max_distance`：根据实际飞行距离设置
- `min_area`/`max_area`：根据相机分辨率和目标大小设置

### 相机参数
- `focal_length`：相机焦距（像素单位）
- 可通过相机标定获得准确值

### 滤波器参数
- `filter_alpha`：0.1 - 0.5
- 值越小，滤波效果越强，响应越慢

## 编译选项

```bash
# 编译所有目标
make all

# 只编译库
make lib

# 编译调试版本
make debug

# 清理编译文件
make clean

# 安装到系统目录
make install

# 卸载
make uninstall
```

## 注意事项

1. **基准四元数**：必须在发射瞬间准确设置
2. **时间同步**：视觉和IMU数据的时间戳需要同步
3. **参数校准**：实际使用前需要根据具体硬件校准参数
4. **实时性**：控制周期建议在10-100ms之间
5. **错误处理**：始终检查输出是否有效

## 技术报告参考

本库基于技术报告中的以下算法实现：
- 广义比例导引律 (Generalized Proportional Navigation)
- 基于像素面积的距离估计
- 四元数姿态处理
- 解耦的俯仰/偏航控制

## 许可证

本项目采用MIT许可证。详见LICENSE文件。

## 支持与贡献

如有问题或建议，请提交Issue或Pull Request。
