# V33 / V34 硬件版本与源码差异

两版是同一套架构长出来的：共 **53 个源文件**（`User/` 下 8 个层目录，目录名完全对应），其中 **只有 10 个文件内容不同**，另有 1 对按键设备被整体替换。共享层的复制粘贴成本很低，板级层各走各的。

## 共同点

- MCU：STM32F405RGT6，LQFP64，1 MB Flash / 192 KB RAM（含 64 KB CCM）
- 主频：SYSCLK = HCLK = 168 MHz，APB1 = /4，APB2 = /2
- CubeMX 工程同名：`A133P_Dart.ioc`
- Keil target 同名：`A133P_Dart`，输出目录 `MDK-ARM/A133P_Dart/`
- `User/` 分层：`Algorithm` `Application` `Common` `Config` `Device` `Driver` `Protocol` `Task`

## 内容有差异的文件

| 文件 | 层 | 说明 |
| --- | --- | --- |
| `Algorithm/alg_closed_loop_controller.cpp` | 共享层 | ⚠️ 闭环控制器参数/逻辑有改动，**改这里必须核对两边** |
| `Application/app_dart_system.cpp` | 板级 | 系统装配（初始化顺序、设备注册） |
| `Config/dart_light_config.h` | 板级 | 灯光的颜色 / 时序配置 |
| `Config/servo_config.cpp` | 板级 | 舵机参数（机械结构变动后需重新标定） |
| `Device/dev_host_serial.cpp/.h` | 共享层 | ⚠️ 上位机串口设备，与协议改动联动 |
| `Protocol/host_serial_protocol.cpp/.h` | 共享层 | ⚠️ IMU 帧 29 → 31 字节（新增 `Image_seq`） |
| `Task/task_bridge.cpp/.h` | 共享层 | ⚠️ 上下位机桥接任务，与协议改动联动 |

带 ⚠️ 的 4 组是**联动修改**：协议一变，`host_serial_protocol`、`dev_host_serial`、`task_bridge` 三者必须同批改，否则会出现编译通过但运行时收不到帧的情况。

## 设备层替换

| V33 | V34 | 含义 |
| --- | --- | --- |
| `Device/dev_key_edge_counter.cpp/.h` | `Device/dev_key_long_press.cpp/.h` | 按键交互从"边沿计数"改成"长按判定"，对应 V34 硬件上的按键改动 |

## 完全一致的部分

`Common`（9 个）、`Driver`（12 个）、`Protocol/dart_light_protocol`、`Algorithm` 其余 5 个、`Device` 其余 11 个。这些文件在两版之间可以直接互拷。

## 维护规则

1. **改共享层，两边一起改，同一个提交里做完**，commit message 写清"V33+V34 同步"。
2. **改板级层，只动自己那一版**，不要顺手改另一边。
3. 这条规则只靠人守不住。等 V33 这一季结束后，把共享层抽到 `firmware/common/`，两个 `.uvprojx` 用 `..\common\` 相对路径引用——那时"两边不一致"在编译期就能暴露。
4. 硬件差异（引脚、外设、机械）请补在本文件下方，比翻代码快得多。
