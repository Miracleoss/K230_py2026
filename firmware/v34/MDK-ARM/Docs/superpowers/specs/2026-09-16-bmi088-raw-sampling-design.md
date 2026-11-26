# BMI088 最小原始采样设计

日期：2026-09-16

## 目标

在已经通过的 BMI088 SPI Probe 基础上，建立一条可在 Keil Watch 中直接验证的最小采样链路。上电后完成加速度计和陀螺仪复位、配置与回读；随后在任务上下文以 200 Hz 轮询读取六轴原始计数和温度。成功时应能看到六轴数值随板子平移、翻转和旋转而变化，采样计数持续递增，最近一次采样结果为 `ResultCode::Ok`。

本阶段只验证传感器初始化和原始数据读取，不把原始计数写入 `ImuData`。`ImuData` 的字段已约定为完成标定、坐标变换后的机体系 SI 单位数据，后续阶段再接入量程换算、轴映射、零偏标定和姿态解算。

## 依据

寄存器、时序和数据格式以 Bosch 官方资料为准：

- *BMI088 Data sheet*, BST-BMI088-DS000-19, Revision 1.9, 2024-01：第 13 页设备初始化，第 15 页电源模式，第 27 页加速度原始数据，第 28 页温度，第 29-30 页加速度配置与量程，第 35 页加速度软复位，第 39-40 页陀螺仪配置与软复位，第 46-48 页 SPI 协议。官方地址：<https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf>
- Bosch Sensortec 官方 `BMI08x_SensorAPI`：`bmi08a.c`、`bmi08g.c` 和 `bmi08_defs.h`。该实现使用加速度软复位后 1 ms、陀螺仪软复位后 30 ms、加速度电源寄存器之间各 5 ms、加速度配置后 40 ms、陀螺仪配置后 10 ms 的等待时间。

当前 SPI1 实际时钟约 2.625 MHz，低于数据手册规定的 10 MHz 上限。SPI 模式、两个片选和芯片 ID 读取已经由实板 Probe 验证。

## 设备层接口

在 `User/Device/dev_bmi088.h` 中增加独立原始样本类型：

```cpp
struct Bmi088RawSample {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature_raw;
    float temperature_deg_c;
};
```

`temperature_raw` 保存解码后的有符号 11 位计数，`temperature_deg_c` 便于 Keil Watch 直接判断数值是否合理。新增接口：

```cpp
typedef void (*Bmi088DelayMsFunction)(uint32_t delay_ms);

ResultCode Initialize(Bmi088DelayMsFunction delay_ms);
ResultCode ReadRawSample(Bmi088RawSample& sample);
bool IsInitialized() const;
```

延时通过回调注入。Device 层因此不依赖 `HAL_Delay()`，主机测试可记录等待时序，任务层则传入一个调用 `HAL_Delay()` 的薄包装。`Initialize()` 只有全部配置及回读成功后才置 `initialized_ = true`；`Bind()` 或任意一次初始化失败都会保持未初始化状态。`ReadRawSample()` 在未初始化时返回 `NotReady`。

内部增加加速度计/陀螺仪的单寄存器写、连续读和回读校验。所有事务继续由 Device 层控制对应 CS。SPI 失败后仍尝试拉高 CS，并优先返回原始 SPI 错误；SPI 成功但 CS 拉高失败时返回 GPIO 错误。

## 初始化配置和时序

初始化使用固定的 bring-up 配置，避免在最小阶段引入配置对象：

| 部件 | 寄存器 | 写入值 | 含义 |
|---|---:|---:|---|
| ACC | `ACC_SOFTRESET` `0x7E` | `0xB6` | 软复位 |
| ACC | `ACC_PWR_CONF` `0x7C` | `0x00` | Active mode |
| ACC | `ACC_PWR_CTRL` `0x7D` | `0x04` | 开启加速度和温度 |
| ACC | `ACC_CONF` `0x40` | `0xA9` | Normal filter，200 Hz ODR，约 40 Hz 截止 |
| ACC | `ACC_RANGE` `0x41` | `0x01` | ±6 g |
| GYRO | `GYRO_SOFTRESET` `0x14` | `0xB6` | 软复位 |
| GYRO | `GYRO_BANDWIDTH` `0x10` | `0x04` | 200 Hz ODR，23 Hz 带宽 |
| GYRO | `GYRO_RANGE` `0x0F` | `0x00` | ±2000 °/s |

顺序如下：

1. 将 `initialized_` 清零，检查绑定和延时回调。
2. 写 ACC 软复位，等待 1 ms；复位会清除 ACC 的 SPI 模式，因此执行一次丢弃结果的 Chip ID 读取重新激活 SPI。
3. 写 GYRO 软复位，等待 30 ms；这段时间不访问陀螺仪。
4. 重新 Probe 两个 Chip ID，确认仍为 `0x1E` 和 `0x0F`。
5. 写 `ACC_PWR_CONF=0x00`，等待 5 ms；写 `ACC_PWR_CTRL=0x04`，再等待 5 ms。
6. 写 `ACC_CONF=0xA9`，等待 40 ms；写 `ACC_RANGE=0x01`，等待 1 ms。
7. 写 `GYRO_BANDWIDTH=0x04`，等待 1 ms；写 `GYRO_RANGE=0x00`，等待 10 ms。
8. 回读所有可读配置寄存器。ACC 按完整字节比较；GYRO 带宽只比较低 3 位，因为位 7 读回恒为 1。任一值不符返回 `InvalidData`。
9. 全部通过后置 `initialized_ = true`。

这些等待时间大于数据手册对复位、电源切换和连续写操作的最小要求，并与 Bosch 官方 SensorAPI 的保守等待一致。初始化只执行一次，因此几十毫秒的阻塞不会进入周期采样路径。

## 原始读取与解析

每个样本包含三次独立 SPI 事务：

1. ACC `0x12..0x17`：发送读地址后发送一个 dummy 字节，再接收 6 个数据字节，总长度 8 字节；丢弃接收缓冲区前两个字节。
2. GYRO `0x02..0x07`：发送读地址后直接接收 6 个数据字节，总长度 7 字节；丢弃接收缓冲区第一个字节。
3. TEMP `0x22..0x23`：按 ACC 规则读取两个数据字节，总长度 4 字节。

六轴数据均按 little-endian 的二补码组合：

```text
axis = int16_t((msb << 8) | lsb)
```

温度按官方 11 位格式解析：

```text
raw11 = (TEMP_MSB << 3) | (TEMP_LSB >> 5)
if raw11 > 1023: raw11 -= 2048
temperature_deg_c = raw11 * 0.125 + 23.0
```

`ReadRawSample()` 先写入局部候选值，三次事务全部成功后才覆盖调用者的样本。这样一次中途失败不会形成 ACC、GYRO 和温度混杂的半更新样本。

## 应用与任务调度

`Class_DartSystem` 只增加 `InitializeBmi088()`、`ReadBmi088RawSample()` 和 `IsBmi088Initialized()` 三个薄转发接口，并继续记录最后错误。寄存器知识不进入 Application 或 Task 层。

`Task_Init()` 保留现有 Probe Watch 变量，并新增：

- `bmi088_init_result`
- `bmi088_sample_result`
- `bmi088_raw_sample`
- `bmi088_sample_count`

Probe 成功后调用初始化。BMI088 初始化失败只停止 BMI088 采样，不阻断当前舵机、串口和 TIM5 bring-up，错误通过独立 Watch 变量可见。

`Task_Loop()` 继续由 TIM5 的 1 kHz 软件时基驱动。新增独立 5 分频计数器，每 5 ms 在任务上下文调用一次 `ReadBmi088RawSample()`：成功时覆盖 `bmi088_raw_sample` 并递增 `bmi088_sample_count`，失败时保留最后一个完整样本并更新 `bmi088_sample_result`。SPI 读取不会放进 TIM5 ISR 或 GPIO EXTI ISR。

现有 `imu_data` 和串口遥测内容本阶段保持原语义，避免把原始计数误标为 m/s² 或 rad/s。

## 错误处理

- 未绑定：`NotReady`。
- 延时回调为空：`ConfigError`。
- HAL SPI 超时/错误：沿用 `Class_SpiBus` 的 `Timeout` 或 `HardwareError`。
- 配置回读不一致或 Chip ID 不符：`InvalidData`。
- 初始化失败：`initialized_` 保持 false，周期采样不执行。
- 周期读取失败：保留最后一个成功样本，采样计数不增加。

## 验证

主机行为测试使用现有 `Tools/TestSupport/stm32f4xx_hal.h` 和真实的 `Class_SpiBus`、`Class_GpioPin`、`Class_Bmi088`：

- 验证 ACC 读事务包含 dummy byte，GYRO 读事务不包含 dummy byte。
- 验证软复位、电源、配置写入顺序和延时序列。
- 验证配置回读成功时进入 initialized 状态，回读不一致时返回 `InvalidData`。
- 验证正负六轴值的 little-endian 解码。
- 验证温度零点、正值和负值的 11 位符号扩展及公式。
- 验证中途 SPI 失败时 CS 恢复为高、输出样本不被部分覆盖。

随后运行 Device/Application/Framework 契约脚本和 Keil ARMCLANG 全量构建。实板验证在 Keil Watch 中观察上述四个变量：静止时某一 ACC 轴应表现出重力，GYRO 三轴应接近零；翻转或绕单轴旋转时相应原始值和符号应改变；温度应在合理范围内，但由于芯片温度寄存器约 1.28 秒更新一次，不要求每个 200 Hz 样本都变化。

## 后续阶段

原始读取通过实板验证后，再单独设计以下内容：确认 PA4 实际连接的 BMI088 中断源并接入 DRDY、原始量程换算、传感器到机体的轴映射、零偏与比例标定、填充 `ImuData`、温控和姿态解算。本阶段不预先实现这些接口。
