# IMU 闭环控制接入：修改前设计审查报告

**状态：实施前基线（方案已获确认并已执行）**  
**审查对象：** 当前 BMI088、主机过载指令、BodyControlCommand、X 尾混控和舵机输出链路  
**目标：** 在批准后接入“主机过载指令 → IMU 实测量 → 外环 PI → 内环 PID → R/P/Y → 舵机分配”。

## 1. 结论摘要

当前工程已经具备可以复用的硬件和执行链路：

1. BMI088 原始数据可换算为 SI 单位，并在唯一边界转换到机体系。
2. ImuData 已提供机体系比力和机体系角速度。
3. 主机过载帧可以校验、解码，并由 Class_HostSerial 提供最新指令。
4. BodyControlCommand、Class_XTailMixer 和四路舵机输出已经连通。
5. 舵机输出非法、未绑定或下发失败时，现有代码会全部回中。

当前缺口：

1. 外环过载 PI 和内环角速度 PID 尚未实现。
2. Task_Loop 仍使用 test_Roll/test_Pitch/test_Yaw，过载指令只进入 output_test。
3. IMU 每约 5 ms 才产生一个新样本，而主循环每 1 ms 进入一次；控制器不能对同一样本重复积分和微分。
4. 现有 BodyControlCommand/X 尾混控使用有符号 [-1,1]，需求要求 ctrl_R/ctrl_P/ctrl_Y 为 [0,1]，需要唯一的中位映射。
5. 主机字段仍为 roll_overload_g/yaw_overload_g/pitch_overload_g，必须显式映射为 n_y_c/n_z_c。
6. 主机指令和 IMU 样本没有陈旧超时策略。

建议保留现有 BodyControlCommand 和 X 尾混控接口，在闭环控制边界增加严格符号适配层，避免重写已经调试过的执行接口。

## 2. 当前实现基线

### 2.1 IMU 和坐标系

User/Common/imu_data.h:13-23 的 ImuData 包含：

| 字段 | 语义 | 单位 |
|---|---|---|
| specific_force_x/y/z_mps2 | 机体系三轴比力 | m/s² |
| gyro_x/y/z_radps | 机体系三轴角速度 | rad/s |
| sample_timestamp_ms | 样本时间戳 | ms |
| data_ready_seq | 成功样本序号 | count |
| valid | 样本有效标志 | bool |

User/Device/dev_bmi088.cpp:29-40 是唯一的传感器到机体转换边界：

    body_x = -sensor_y
    body_y =  sensor_z
    body_z = -sensor_x

角速度使用同样的轴映射。Bmi088RawSampleToImuData 先换算量纲，再调用该转换。因此闭环只能直接消费 bmi088_body_imu_data，不能再次换轴或取负号。

BMI088 原始换算已有 9.80665 m/s² 常量；需求的过载公式写明使用 g=9.81 m/s²。建议在新增控制配置中定义 kControlGravityMps2 = 9.81f，不改动已有原始采样换算。

这里按需求把机体系比力直接除以 g 作为实测过载。比力在物理上包含重力/表观加速度项，既有 Stage1 架构文档曾把“直接把比力当过载”列为待确认事项；本方案把它视为本次闭环的明确控制假设，实施前应通过静态和动态实测确认符号及零点。

角速度的内部统一单位仍是 rad/s。对外遥测要区分两种现有模式：普通 Host 固定帧在 `host_serial_protocol.cpp:231-238` 将 rad/s 转为 0.1 deg/s/LSB 后发送；VOFA+ JustFloat 在 `host_serial_protocol.cpp:267-275` 直接发送 rad/s 浮点值。

### 2.2 主机过载指令

User/Protocol/host_serial_protocol.cpp:193-215 当前解码 14 字节 AA 55 ... CRC 0D 0A 帧：

| 帧位置 | 当前字段 | 单位 |
|---:|---|---|
| 2 | roll_overload_g | g |
| 4 | yaw_overload_g | g |
| 6 | pitch_overload_g | g |

Class_HostSerial::Poll/TakeLatestOverload 位于 User/Device/dev_host_serial.cpp:30-75。解析器只发布校验通过的最新帧，没有年龄或超时状态。

闭环适配必须明确为：

    n_y_c = HostOverloadCommand.pitch_overload_g
    n_z_c = HostOverloadCommand.yaw_overload_g
    omega_x_c = 0

现有 roll_overload_g 不进入本闭环外环。第一版建议保留线上帧格式，将该字段作为兼容字段读取但不参与控制；是否删除它需要单独批准。

### 2.3 现有 RPY 执行链路

User/Common/control_command.h:10-22 的 BodyControlCommand 字段为 roll/pitch/yaw，应用层要求范围 [-1,+1]。

User/Algorithm/alg_xtail_mixer.cpp:7-40 当前矩阵为：

    delta1 = roll + yaw + pitch
    delta2 = roll + yaw - pitch
    delta3 = roll - yaw - pitch
    delta4 = roll - yaw + pitch

结果按最大绝对值整体缩放到 [-1,+1]，再由 Class_Servo::SetDelta 映射到 PWM。User/Application/app_dart_system.cpp:78-131 负责校验、混控、逐路下发和失败回中。

这个接口的物理零点是三轴 0。因此 ctrl_R/P/Y 不能直接传入：本方案规定 [0,1] 的 0.5 为中位，最终通过 2*ctrl-1 转成现有有符号接口。

### 2.4 当前任务时序

Core/Src/main.c:104-116 在死循环中调用 Task_Loop。User/Task/task_bridge.cpp:341-391 当前流程为：

1. TIM5 时间基去重，同一毫秒只执行一次；
2. 每 5 个毫秒节拍读取一次 BMI088，约 200 Hz；
3. 保存机体系 ImuData 并发送遥测；
4. 轮询主机串口；
5. 用测试变量构造 BodyControlCommand 并下发；
6. 最后才取走过载指令，并只调用 output_test。

正式闭环应改为：主机轮询/取指令 → 读取或确认新 IMU 样本 → 仅在新样本到达时运行一次控制器 → 生成 BodyControlCommand → 下发舵机。

## 3. 目标数据流

    主机 AA55 过载帧
      -> 协议解码
      -> n_y_c / n_z_c（滚转字段仅兼容）

    BMI088 原始样本
      -> SI 换算 + 唯一机体系转换
      -> ImuData（f_x/f_y/f_z、omega_x_meas/omega_y_meas/omega_z_meas）

    n_y_c/n_z_c + f_y/f_z
      -> 过载 PI 外环
      -> omega_x_c=0、omega_y_c、omega_z_c

    角速度指令 + omega_x_meas/y_meas/z_meas
      -> R/P/Y 角速度 PID 内环
      -> ctrl_R/ctrl_P/ctrl_Y（0..1）
      -> signed_R/P/Y = 2*ctrl-1
      -> BodyControlCommand
      -> Class_XTailMixer
      -> SurfaceCommand
      -> 四路 Servo PWM

第一版不把 Class_AttitudeEstimator 插入速率闭环。需求要求内环直接使用 IMU 机体系角速度，而 User/Algorithm/alg_attitude_estimator.cpp:3-5 目前只有构造函数。这里的 R/P/Y 是三个控制轴输出，不要求先计算欧拉姿态角。

## 4. 控制量和公式

| 变量 | 含义 | 单位/范围 | 来源 |
|---|---|---|---|
| n_y_c | 俯仰期望过载 | g 倍数、无量纲 | 主机 pitch 字段 |
| n_z_c | 偏航期望过载 | g 倍数、无量纲 | 主机 yaw 字段 |
| f_x/f_y/f_z | IMU 测量比力 | m/s² | ImuData.specific_force_* |
| n_y_meas/n_z_meas | 实测过载 | 无量纲 | 比力除以 g |
| e_ny/e_nz | 外环误差 | 无量纲 | n_*_c - n_*_meas |
| omega_x_c/y_c/z_c | 角速度指令 | rad/s | 外环输出 |
| omega_x_meas/y_meas/z_meas | 实测角速度 | rad/s | ImuData.gyro_*_radps |
| e_omega_x/y/z | 内环误差 | rad/s | omega_*_c - omega_*_meas |
| ctrl_R/P/Y | 最终 R/P/Y 控制量 | [0,1] | 内环 PID 输出 |

### 4.1 外环 PI

严格采用需求公式：

    n_y_meas = f_y / kControlGravityMps2
    n_z_meas = f_z / kControlGravityMps2
    e_ny = n_y_c - n_y_meas
    e_nz = n_z_c - n_z_meas
    omega_x_c = 0
    omega_y_c = PI_y(e_ny, dt_s)
    omega_z_c = PI_z(e_nz, dt_s)

建议并联 PI：

    integral_y += e_ny * dt_s
    omega_y_c = clamp(Kp_ny*e_ny + Ki_ny*integral_y,
                      omega_y_min, omega_y_max)

偏航通道同理。输出饱和时采用条件积分或回算 anti-windup。当前工程没有任何可复用增益，数值必须由调参数据提供。

主机协议当前只做 int16/100 解码，没有业务范围校验；可编码范围约为 ±327.67 g。闭环入口还必须对 n_y_c/n_z_c 做配置限幅，并明确超限时是拒绝该帧、饱和后使用，还是直接进入安全输出，不能把异常指令直接交给 PI。

### 4.2 内环 PID

    e_omega_x = omega_x_c - omega_x_meas
    e_omega_y = omega_y_c - omega_y_meas
    e_omega_z = omega_z_c - omega_z_meas

建议三个轴独立保存积分和上次误差：

    integral += error * dt_s
    derivative = (error - previous_error) / dt_s
    u_signed = Kp*error + Ki*integral + Kd*derivative
    u_signed = clamp(u_signed, -1, +1)
    ctrl = clamp(0.5 + 0.5*u_signed, 0, 1)

为减少角速度指令阶跃导致的 derivative kick，默认建议 D 项对实测角速度求导并增加一阶低通；这属于实现选项，需要审查确认。

### 4.3 与现有 BodyControlCommand 的转换

    command.roll  = 2 * ctrl_R - 1
    command.pitch = 2 * ctrl_P - 1
    command.yaw   = 2 * ctrl_Y - 1

对应关系：

| ctrl_* | 现有字段 | 意义 |
|---:|---:|---|
| 0.0 | -1.0 | 负向满量程 |
| 0.5 | 0.0 | 中位/零控制 |
| 1.0 | +1.0 | 正向满量程 |

故障或控制器复位时输出 ctrl_R=ctrl_P=ctrl_Y=0.5，转换为 BodyControlCommand 三轴 0，复用现有舵机回中保护。

## 5. 建议的模块和文件变更

以下是批准后才执行的候选变更：

### 5.1 新增纯算法模块

建议新增：

    User/Algorithm/alg_closed_loop_controller.h
    User/Algorithm/alg_closed_loop_controller.cpp

职责：

1. 保存 Y/Z 两个过载 PI 和 R/P/Y 三个角速度 PID 的状态；
2. 接收 n_y_c/n_z_c、ImuData 和 dt_s；
3. 输出 n_y_meas/n_z_meas、omega_x_c/y_c/z_c 和 ctrl_R/P/Y；
4. 提供 Reset，清除积分、微分和首次样本状态；
5. 不依赖 HAL、串口、BMI088 设备或舵机。

为减少集成面，第一版建议先把 PI/PID 状态放进闭环控制类内部；若拆成 class_pid.*，需要同步更新旧的 Tools/verify_framework_contract.ps1 禁止规则。

### 5.2 Class_DartSystem

保留 ApplyBodyControlCommand(const BodyControlCommand&)。建议由 Class_DartSystem 聚合闭环算法对象，并增加一个应用层入口（名称待定），形式类似：

    UpdateClosedLoop(n_y_c, n_z_c, imu, dt_s, command)

该入口内部调用纯算法模块，再把 ctrl_R/P/Y 转成现有 BodyControlCommand，最后仍由 ApplyBodyControlCommand 负责混控和舵机保护。主机指令锁存、接收时间和超时判断留在 Task 层；PI/PID 的积分状态归闭环算法对象所有，避免 Task 直接持有应用算法或绕过 Class_DartSystem 的聚合边界。

### 5.3 Task_Loop

建议顺序：

1. 每 1 ms 做调度和串口轮询；
2. Poll 后 TakeLatestOverload，更新锁存的 n_y_c/n_z_c 和 last_host_command_ms；
3. BMI088 成功采样后设置 new_imu_sample；
4. 只有新样本到达才运行一次 PI/PID；
5. 转换成 BodyControlCommand 后调用 ApplyBodyControlCommand；
6. 样本无效、超时或输出非法时重置控制器并令命令无效；
7. 删除 output_test 对测试变量的控制作用。

### 5.4 控制配置

建议新增 User/Config/control_config.h，集中放置：

- kControlGravityMps2；
- 外环 PI 增益、积分限幅和角速度限幅；
- 内环 R/P/Y PID 增益、积分限幅、输出限幅和 D 滤波系数；
- IMU 陈旧超时、主机指令超时和采样周期容差；
- [0,1] 输出的中位与映射常量。

Keil 工程 MDK-ARM/A133P_Dart.uvprojx 使用显式文件列表。新增源文件后必须加入对应 source group，否则 IDE 构建不会编译它们。

## 6. 时序、有效性和失效保护

### 6.1 控制周期

TIM5 约 1 ms，BMI088 轮询约 5 ms；第一版控制频率应锁定为每个新 IMU 样本一次，约 200 Hz。dt_s 使用相邻有效 sample_timestamp_ms 的差值乘 0.001f。

以下情况不推进积分或微分：

- ImuData.valid == false；
- sample_timestamp_ms 未变化；
- 时间戳回退或 dt_s <= 0；
- dt_s 超过配置容差；
- 任一输入或输出不是有限浮点数。

### 6.2 主机指令锁存和超时

有效新帧到达后锁存 n_y_c/n_z_c，并记录 last_host_command_ms。超过 kHostCommandTimeoutMs 后清除控制器状态、输出中位命令并记录超时。具体超时值当前没有提供，不能擅自猜测。

### 6.3 IMU 陈旧

连续读样本失败或 now_ms - sample_timestamp_ms 超过 kImuStaleTimeoutMs 时，进入同样的安全输出。现有 ResultCode 和舵机回中路径继续作为最终保护。

### 6.4 故障路径

    输入/时序异常
      -> controller.Reset()
      -> BodyControlCommand.valid = false
      -> ApplyBodyControlCommand()
      -> SetAllServosNeutral()

不在中断中执行 SPI、PID、混控或四路 PWM 下发；TIM5 ISR 继续只维护时间基。

现有 SystemStatus 和 FaultCode 已定义若干 IMU/主机/控制器故障位，但当前工程主要只更新 last_error。若需要把 IMU 陈旧、主机超时和控制器无效状态送到遥测或 Watch，实施阶段还要补充状态更新和读取接口；这不改变回中主路径。

## 7. 复用项和第一版不纳入项

### 直接复用

- Bmi088RawSampleToImuData 的量纲换算和唯一坐标转换；
- ImuData 的时间戳、序号和 valid；
- Class_HostSerial::Poll、TakeLatestOverload；
- BodyControlCommand 的 RPY 执行接口；
- Class_XTailMixer 矩阵和整体归一化；
- Class_Servo::SetDelta 的限幅、反向、零位和 PWM 映射；
- ApplyBodyControlCommand 的绑定检查和全部回中；
- TIM5 1 ms 时间基。

### 第一版不纳入

- 不重写 BMI088 驱动和坐标映射；
- 不把原始 Bmi088RawSample 送入控制器；
- 不实现或强行调用当前空壳 Class_AttitudeEstimator；
- 不改变主机帧头、长度、CRC 和字节顺序；
- 不修改舵机分配矩阵；
- 不在中断中运行控制算法。

## 8. 需要审查确认的决策

1. 是否确认 [0,1] 中 0.5 是零控制，并按 2*ctrl-1 映射到现有 BodyControlCommand？
2. 是否保持现有帧格式并忽略滚转过载字段，而不修改上位机协议？
3. 是否严格使用 9.81f 作为过载除数？
4. 外环 Y/Z 的 Kp/Ki、角速度限幅是多少？
5. 内环 R/P/Y 的 Kp/Ki/Kd、积分限幅、输出限幅和 D 滤波参数是多少？
6. 主机指令和 IMU 陈旧超时是多少，超时后是否始终回中？
7. D 项使用误差微分，还是测量量微分加低通？默认建议后者。
8. 是否必须收到第一条有效主机过载指令后才允许控制？

## 9. 实施后检查清单

1. 比力变量只使用 f_x/f_y/f_z 语义，不引入 a 表示比力；指令用 _c，测量用 _meas，输出用 ctrl_R/P/Y。
2. 主机映射为 pitch → n_y_c、yaw → n_z_c。
3. 单位保持 m/s²、g 倍数、rad/s 和秒。
4. 不重复执行坐标变换。
5. 同一 data_ready_seq 不重复运行 PI/PID。
6. ctrl_*=0.5 能回到 BodyControlCommand 三轴 0 和舵机中位。
7. 所有控制器和舵面输出都有明确限幅。
8. IMU 无效、主机超时、NaN、舵机下发失败都会回中并复位积分。
9. 新源文件加入 A133P_Dart.uvprojx，旧契约脚本同步更新。

## 10. 修改前基线记录

- 本节内容记录方案形成时的工作区状态。
- 实际实现结果和验证记录见下一节。

## 11. 批准后的实施状态

本报告形成后已按审查结论完成接入，设计中的“待执行”描述保留作为修改前基线；当前实现状态如下：

1. 新增 `User/Config/control_config.h`、`User/Algorithm/alg_closed_loop_controller.h/.cpp`，实现 Y/Z 过载 PI、R/P/Y 角速度 PID、积分限幅、输出限幅、样本门控和复位。
2. `Class_DartSystem::UpdateClosedLoop()` 聚合纯控制器，并将 `ctrl_R/P/Y` 按 `2*ctrl-1` 映射为现有 `BodyControlCommand`；`ResetClosedLoop()` 用于超时和无效路径。
3. `Task_Loop` 已按 `Poll → TakeLatestOverload → 新 IMU 样本 → 闭环更新 → ApplyBodyControlCommand` 顺序运行。主机字段映射为 `pitch_overload_g → n_y_c`、`yaw_overload_g → n_z_c`，滚转过载字段不进入外环。
4. 控制器内部角速度统一使用 `rad/s`。普通 Host 遥测仍编码为 `0.1 deg/s/LSB`，VOFA+ JustFloat 仍发送 `rad/s` 浮点值。
5. 默认增益保持零值，硬件调参前输出保持中位；默认时序保护为 `dt=1..20 ms`、主机超时 100 ms、IMU 陈旧 20 ms。上述非增益参数属于安全默认值，正式飞行前仍需依据调参数据确认。
6. 新文件已登记到 `A133P_Dart.uvprojx` 和 `.eide/eide.yml`，并新增 `Tools/verify_closed_loop_controller.ps1`。

已执行的验证脚本：

- `verify_closed_loop_controller.ps1`
- `verify_algorithm_contract.ps1`
- `verify_application_contract.ps1`
- `verify_framework_contract.ps1`
- `verify_driver_contract.ps1`
- `verify_device_contract.ps1`
- `verify_host_serial_protocol.ps1`
