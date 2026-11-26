# STM32F405 Dart Flight Controller - Stage 0 Project Audit

> Scope: project audit only. No source code, `.ioc`, Keil project, EIDE project, or firmware logic was modified.
>
> Audit date: 2026-08-30
>
> Audit working directory: `D:\Work_stm\ZLLCProject\A133P_Dart\MDK-ARM`
>
> Actual CubeMX project root observed: `D:\Work_stm\ZLLCProject\A133P_Dart`

## 1. Executive Summary

当前工程完成度：工程当前基本处于 CubeMX/HAL 外设初始化阶段。已经生成并纳入 Keil/EIDE 工程的内容包括 `GPIO`、`DMA`、`SPI1`、`USART1`、`TIM1`、`TIM3`、`TIM5`、中断入口和 HAL/CMSIS 驱动。没有发现 BMI088 驱动、舵机抽象、A133P 协议、姿态解算、PID、混控、任务调度、飞控状态机或 failsafe 业务逻辑。

当前工程是否适合直接在其上建立新架构：适合作为 Stage 1 架构设计的硬件配置基线，但不适合直接认为任何飞控功能已经可用。`Core/` 仍主要是 CubeMX 自动生成代码，用户业务层目录 `User/` 尚不存在，`main()` 初始化后 `while(1)` 为空。

最大的三个问题：

| # | 问题 | 证据 |
|---|---|---|
| 1 | TIM1 四路 PWM 和 TIM3 CH3 PWM 已配置但未启动，当前不会输出 PWM。 | `Core/Src/main.c:92-98` 只调用 MX init；未发现 `HAL_TIM_PWM_Start`；`Core/Src/tim.c:63,143` 只初始化 PWM。 |
| 2 | PA4 `Data_Ready_INT` 配成 EXTI 上升沿，但未启用 EXTI4 NVIC，也没有 `EXTI4_IRQHandler`。 | `.ioc:101-104`、`Core/Src/gpio.c:63-66`；`Core/Src/stm32f4xx_it.c` 中无 EXTI4 handler。 |
| 3 | USART1 配置了 DMA RX/TX 和 IRQ，但没有任何接收启动、IDLE Line、缓冲区或协议解析代码。 | `Core/Src/usart.c:87-120` 配 DMA；`Core/Src/main.c:99-110` 无用户启动代码；全文未发现 `HAL_UARTEx_RxEventCallback`、接收 buffer 或协议代码。 |

最大的三个未知信息：

| # | 未知信息 | 当前结论 |
|---|---|---|
| 1 | 四个 TIM1 通道到物理舵机/舵面的 Servo1~4 对应关系。 | 无法从当前工程确认。 |
| 2 | BMI088 只看到两个 CS 和一个 `Data_Ready_INT`，无法确认该中断属于 ACC、GYRO，或是否还有另一路 INT。 | 无法从当前工程确认。 |
| 3 | A133P 通信协议、帧格式、CRC 类型、超时/failsafe 行为、控制周期和舵机脉宽范围。 | 待后续 Stage 设计。 |

## 2. Project Structure

项目实际源码根目录是 `A133P_Dart/`，Keil 工程目录是 `A133P_Dart/MDK-ARM/`。当前 `MDK-ARM` 目录下存在编译产物和 IDE 配置，源码位于父目录 `Core/` 和 `Drivers/`。

| 路径 | 类型 | 作用 | 是否 CubeMX 生成 | 后续是否可能保留 |
|---|---|---|---|---|
| `../A133P_Dart.ioc` | CubeMX 配置 | MCU、引脚、外设、时钟、DMA、NVIC 配置源 | 是 | 是，作为硬件配置基线 |
| `../.mxproject` | CubeMX 元数据 | 记录生成文件、Keil 源文件、头文件和包信息 | 是 | 是，但通常由 CubeMX 管理 |
| `../Core/Inc/*.h` | 头文件 | HAL 初始化接口、句柄声明、引脚宏、配置 | 是 | 是，继续由 CubeMX 管理 |
| `../Core/Src/*.c` | 源文件 | `main`、GPIO、DMA、SPI、TIM、USART、IRQ、MSP、system | 是 | 是，继续作为 HAL/CubeMX 层 |
| `../Drivers/STM32F4xx_HAL_Driver/` | HAL 驱动 | ST HAL/LL 驱动库 | 是/库文件 | 是 |
| `../Drivers/CMSIS/` | CMSIS 驱动/库 | CMSIS Core、Device、DSP、NN 等 | 是/库文件 | 部分保留；未被工程引用的大量 CMSIS 示例可后续评估 |
| `A133P_Dart.uvprojx` | Keil 工程 | MDK-ARM V5 工程定义，引用 `Core` 与 HAL 源文件 | 是/工具生成 | 是 |
| `A133P_Dart.uvoptx` | Keil 用户选项 | 调试器、文件树显示、扩展名等 | 工具生成 | 是 |
| `.eide/eide.yml` | EIDE 配置 | EIDE 项目文件，引用同一批 `Core` 和 HAL 源文件 | 工具生成 | 可保留 |
| `.eide/files.options.yml` | EIDE 文件选项 | 单文件编译选项追加表，当前基本为空 | 工具生成 | 可保留 |
| `.vscode/tasks.json` | VSCode 任务 | EIDE build/flash/rebuild/clean 任务入口 | 用户/工具配置 | 可保留 |
| `.cmsis/` | CMSIS Pack 缓存 | 包含大量 ARM Cortex 示例和通用 CMSIS 资源 | 工具缓存 | 可评估是否需要纳入版本控制 |
| `A133P_Dart/` | 构建输出目录 | `.o/.d/.crf/.axf/.hex/.map/.htm` 等构建产物 | 构建产物 | 通常不应作为源码保留 |
| `build/` | 构建输出目录 | EIDE 输出目录 | 构建产物 | 通常不应作为源码保留 |
| `DebugConfig/` | 调试配置 | Keil/调试配置文件 | 工具生成 | 可保留 |
| `RTE/` | Keil RTE | CMSIS RTE 组件头 | 工具生成 | 可保留 |
| `User/` | 用户业务层 | 未发现该目录 | 不存在 | Stage 1 设计后再创建 |
| `Middlewares/` | 中间件 | 未发现该目录 | 不存在 | 当前不需要 |
| `CMakeLists.txt` / `Makefile` | 构建系统 | 仅在 `Drivers/CMSIS` 示例中存在；项目根未发现 | 否 | 项目当前不使用 |

Keil 工程实际纳入编译的应用文件见 `MDK-ARM/A133P_Dart.uvprojx:398-639`，主要为：

```text
../Core/Src/main.c
../Core/Src/gpio.c
../Core/Src/dma.c
../Core/Src/spi.c
../Core/Src/tim.c
../Core/Src/usart.c
../Core/Src/stm32f4xx_it.c
../Core/Src/stm32f4xx_hal_msp.c
../Core/Src/system_stm32f4xx.c
startup_stm32f405xx.s
```

`Core/Src` 和 `Core/Inc` 中所有 `USER CODE` 区域目前基本为空；没有发现独立用户业务模块。

## 3. MCU & Clock Tree

| 项目 | 当前配置 | 证据 |
|---|---|---|
| MCU CPN | `STM32F405RGT6` | `A133P_Dart.ioc:31` |
| CubeMX MCU Name | `STM32F405RGTx` | `A133P_Dart.ioc:43` |
| Package | `LQFP64` | `A133P_Dart.ioc:44` |
| Keil Device | `STM32F405RGTx` | `MDK-ARM/A133P_Dart.uvprojx:17` |
| HCLK / AHB | 168 MHz | `A133P_Dart.ioc:155-166` |
| SYSCLK | PLLCLK 168 MHz | `A133P_Dart.ioc:180-181` |
| APB1 PCLK | 42 MHz | `A133P_Dart.ioc:156-158` |
| APB1 Timer Clock | 84 MHz | `A133P_Dart.ioc:158` |
| APB2 PCLK | 84 MHz | `A133P_Dart.ioc:159-160` |
| APB2 Timer Clock | 168 MHz | `A133P_Dart.ioc:161` |
| SysTick | CubeMX virtual SysTick | `A133P_Dart.ioc:54,83` |

实际生成代码中，系统时钟使用 HSI 作为 PLL 源：

```text
HSI = 16 MHz
PLLM = 8
PLLN = 168
PLLP = 2
SYSCLK = 16 MHz / 8 * 168 / 2 = 168 MHz
PLLQ = 4 -> 84 MHz
```

证据：`Core/Src/main.c:131-139` 使用 `RCC_OSCILLATORTYPE_HSI`、`RCC_PLLSOURCE_HSI`、`PLLM=8`、`PLLN=168`、`PLLP=DIV2`、`PLLQ=4`；`Core/Src/main.c:149-154` 选择 PLLCLK、AHB DIV1、APB1 DIV4、APB2 DIV2、FLASH latency 5。

注意：`.ioc` 同时把 `PH0/PH1` 配为 HSE oscillator 引脚，见 `A133P_Dart.ioc:118-121`，但生成的 `SystemClock_Config()` 实际没有启用 HSE，而是启用 HSI。该状态应在 Stage 1 前确认硬件是否确实需要外部晶振作为主时钟。

## 4. Hardware Resource Map

| 功能 | STM32 资源 | 引脚 | CubeMX 配置 | 当前代码使用情况 | 结论 |
|---|---|---|---|---|---|
| BMI088 ACC CS | GPIO Output | PA2 | label `BMI088_ACC_CS` | `gpio.c` 初始化为推挽输出，初始写 RESET | 有硬件 GPIO 骨架，无驱动 |
| BMI088 GYRO CS | GPIO Output | PA3 | label `BMI088_GYRO_CS` | `gpio.c` 初始化为推挽输出，初始写 RESET | 有硬件 GPIO 骨架，无驱动 |
| BMI088 Data Ready | EXTI line 4 | PA4 | label `Data_Ready_INT`，`GPXTI4` | `gpio.c` 配上升沿 EXTI，但无 NVIC/IRQ handler | 配置不完整 |
| BMI088 SPI | SPI1 | PA5 SCK, PA6 MISO, PA7 MOSI | Full Duplex Master | `spi.c` 初始化 SPI1；未发现传输调用 | 有 SPI 外设骨架，无 BMI088 驱动 |
| BMI088 Heater | TIM3 CH3 PWM | PB0 | PWM Generation CH3 | `tim.c` 初始化 PWM；未发现 PWM Start 或温控代码 | 有 PWM 骨架，未运行 |
| Servo PWM 1 | TIM1 CH1 PWM | PA8 | PWM Generation CH1 | `tim.c` 初始化 PWM；未发现 PWM Start 或舵机代码 | 有 PWM 骨架，未运行 |
| Servo PWM 2 | TIM1 CH2 PWM | PA9 | PWM Generation CH2 | 同上 | 有 PWM 骨架，未运行 |
| Servo PWM 3 | TIM1 CH3 PWM | PA10 | PWM Generation CH3 | 同上 | 有 PWM 骨架，未运行 |
| Servo PWM 4 | TIM1 CH4 PWM | PA11 | PWM Generation CH4 | 同上 | 有 PWM 骨架，未运行 |
| A133P UART | USART1 | PB6 TX, PB7 RX | Asynchronous | `usart.c` 初始化 UART、DMA、IRQ；未启动收发 | 有 UART/DMA 骨架，无协议 |
| USART1 RX DMA | DMA2 Stream2 Channel4 | N/A | Normal mode, low priority | `usart.c` link 到 `huart1.hdmarx` | 已配置，未启动 |
| USART1 TX DMA | DMA2 Stream7 Channel4 | N/A | Normal mode, low priority | `usart.c` link 到 `huart1.hdmatx` | 已配置，未启动 |
| Periodic timer candidate | TIM5 update IRQ | N/A | PSC=168, ARR=1000 | 初始化并启用 NVIC，但未 start interrupt | 有定时器骨架，未运行 |
| Debug | SWD | PA13 SWDIO, PA14 SWCLK | Serial Wire | `.ioc` 配置 SWD | 未发现冲突 |

## 5. Peripheral Configuration

### SPI1 / BMI088 candidate

| 参数 | 当前值 | 证据 |
|---|---|---|
| Instance | `SPI1` | `Core/Src/spi.c:40` |
| Mode | `SPI_MODE_MASTER` | `Core/Src/spi.c:41` |
| Direction | `SPI_DIRECTION_2LINES` | `Core/Src/spi.c:42` |
| DataSize | `SPI_DATASIZE_8BIT` | `Core/Src/spi.c:43` |
| CPOL | `SPI_POLARITY_HIGH` | `Core/Src/spi.c:44` |
| CPHA | `SPI_PHASE_2EDGE` | `Core/Src/spi.c:45` |
| NSS | `SPI_NSS_SOFT` | `Core/Src/spi.c:46` |
| Prescaler | `SPI_BAUDRATEPRESCALER_16` | `Core/Src/spi.c:47` |
| FirstBit | `SPI_FIRSTBIT_MSB` | `Core/Src/spi.c:48` |
| SPI CRC | disabled | `Core/Src/spi.c:50` |
| GPIO AF | PA5/PA6/PA7, `GPIO_AF5_SPI1` | `Core/Src/spi.c:80-84` |
| GPIO Speed | `GPIO_SPEED_FREQ_VERY_HIGH` | `Core/Src/spi.c:83` |
| DMA | none configured for SPI1 | `.ioc` DMA requests only USART1 RX/TX at lines 5-27 |
| Interrupt | none configured for SPI1 | `.ioc` NVIC lines 72-86 do not list SPI1 |

SPI1 theoretical clock: SPI1 is on APB2, PCLK2 = 84 MHz. Prescaler = 16, so SPI clock = 84 MHz / 16 = 5.25 Mbit/s. This matches `A133P_Dart.ioc:201`.

BMI088 current code use: no `HAL_SPI_Transmit`, `HAL_SPI_Receive`, `HAL_SPI_TransmitReceive`, register definitions, initialization sequence, or BMI088 driver module were found in project application code.

### TIM3 CH3 / BMI088 Heater

| 参数 | 当前值 | 证据 |
|---|---|---|
| Instance | `TIM3` | `Core/Src/tim.c:128` |
| Channel | `TIM_CHANNEL_3` | `Core/Src/tim.c:157` |
| GPIO | PB0 | `A133P_Dart.ioc:113`, `Core/Src/tim.c:286` |
| GPIO AF | `GPIO_AF2_TIM3` | `Core/Src/tim.c:290` |
| GPIO Speed | `GPIO_SPEED_FREQ_LOW` | `Core/Src/tim.c:289` |
| Prescaler | 0 | `Core/Src/tim.c:129` |
| Period / ARR | 65535 | `Core/Src/tim.c:131` |
| ClockDivision | DIV1 | `Core/Src/tim.c:132` |
| PWM mode | `TIM_OCMODE_PWM1` | `Core/Src/tim.c:153` |
| Initial CCR / Pulse | 0 | `Core/Src/tim.c:154` |
| Polarity | `TIM_OCPOLARITY_HIGH` | `Core/Src/tim.c:155` |
| PWM Start | not found | no `HAL_TIM_PWM_Start` match |
| Heater PID / target / limit | not found | no user code found |

TIM3 is on APB1. Because APB1 prescaler is `/4`, timer clock is doubled from PCLK1 42 MHz to 84 MHz. PWM frequency:

```text
84,000,000 / (0 + 1) / (65535 + 1) = 1281.73828125 Hz
```

Current heater PWM output state: configured but not started, CCR=0.

### TIM1 CH1~CH4 / Servo PWM

| 参数 | 当前值 | 证据 |
|---|---|---|
| Instance | `TIM1` | `Core/Src/tim.c:47` |
| Channels | CH1, CH2, CH3, CH4 | `.ioc:206-210`, `Core/Src/tim.c:80-92` |
| GPIO | PA8, PA9, PA10, PA11 | `.ioc:111-112`, `.ioc:87-88`, `Core/Src/tim.c:265` |
| GPIO AF | `GPIO_AF1_TIM1` | `Core/Src/tim.c:269` |
| GPIO Speed | `GPIO_SPEED_FREQ_LOW` | `Core/Src/tim.c:268` |
| Prescaler | 0 | `Core/Src/tim.c:48` |
| Period / ARR | 65535 | `Core/Src/tim.c:50` |
| Counter mode | up | `Core/Src/tim.c:49` |
| ClockDivision | DIV1 | `Core/Src/tim.c:51` |
| RepetitionCounter | 0 | `Core/Src/tim.c:52` |
| PWM mode | `TIM_OCMODE_PWM1` | `Core/Src/tim.c:73` |
| Initial CCR / Pulse | 0 | `Core/Src/tim.c:74` |
| Polarity | `TIM_OCPOLARITY_HIGH` | `Core/Src/tim.c:75` |
| PWM Start | not found | no `HAL_TIM_PWM_Start` match |
| Direct CCR writes | not found | no application-level CCR write found |

TIM1 is on APB2. Because APB2 prescaler is `/2`, timer clock is doubled from PCLK2 84 MHz to 168 MHz. PWM frequency:

```text
168,000,000 / (0 + 1) / (65535 + 1) = 2563.4765625 Hz
```

PWM theoretical period:

```text
1 / 2563.4765625 = 0.000390095238 s = 390.095 us
```

Servo PWM resource table:

| Servo | Timer Channel | GPIO | PWM frequency | Current CCR | Current state |
|---|---|---|---:|---:|---|
| 尚未定义 Servo1 | TIM1 CH1 | PA8 | 2563.4766 Hz | 0 | Configured, not started |
| 尚未定义 Servo2 | TIM1 CH2 | PA9 | 2563.4766 Hz | 0 | Configured, not started |
| 尚未定义 Servo3 | TIM1 CH3 | PA10 | 2563.4766 Hz | 0 | Configured, not started |
| 尚未定义 Servo4 | TIM1 CH4 | PA11 | 2563.4766 Hz | 0 | Configured, not started |

Physical servo-to-tail-surface mapping: 无法从当前工程确认。

### TIM5 / Periodic Timer Candidate

| 参数 | 当前值 | 证据 |
|---|---|---|
| Instance | `TIM5` | `Core/Src/tim.c:181` |
| Prescaler | 168 | `Core/Src/tim.c:182`, `.ioc:215` |
| Period / ARR | 1000 | `Core/Src/tim.c:184`, `.ioc:214` |
| Clock source | internal | `.ioc:224-225` |
| NVIC | enabled, priority 0/0 | `Core/Src/tim.c:242-243`, `.ioc:84` |
| Start IT | not found | no `HAL_TIM_Base_Start_IT` match |
| Period callback | not found | no `HAL_TIM_PeriodElapsedCallback` match |

TIM5 is on APB1, timer clock = 84 MHz. Update frequency if started:

```text
84,000,000 / (168 + 1) / (1000 + 1) = 496.535 Hz
```

Because no start call exists, TIM5 currently does not provide a running scheduler.

### USART1 / A133P

| 参数 | 当前值 | 证据 |
|---|---|---|
| Instance | `USART1` | `Core/Src/usart.c:43` |
| Baudrate | 115200 | `Core/Src/usart.c:44` |
| Word length | 8 bit | `Core/Src/usart.c:45` |
| Stop bits | 1 | `Core/Src/usart.c:46` |
| Parity | none | `Core/Src/usart.c:47` |
| Mode | TX/RX | `Core/Src/usart.c:48` |
| HW flow control | none | `Core/Src/usart.c:49` |
| Oversampling | 16 | `Core/Src/usart.c:50` |
| GPIO | PB6 TX, PB7 RX | `.ioc:114-117`, `Core/Src/usart.c:78` |
| GPIO AF | `GPIO_AF7_USART1` | `Core/Src/usart.c:82` |
| GPIO Speed | `GPIO_SPEED_FREQ_VERY_HIGH` | `Core/Src/usart.c:81` |
| DMA RX | DMA2 Stream2 Channel4, normal, low priority | `.ioc:5-17`, `Core/Src/usart.c:87-102` |
| DMA TX | DMA2 Stream7 Channel4, normal, low priority | `.ioc:18-27`, `Core/Src/usart.c:105-120` |
| USART IRQ | enabled, priority 0/0 | `.ioc:85`, `Core/Src/usart.c:123-124` |
| IDLE Line | not found | no `HAL_UARTEx_ReceiveToIdle_*` or IDLE handling found |
| RX Buffer | not found | no application buffer found |
| Protocol parser | not found | no application parser found |
| printf redirect | not found | no `fputc` / `__io_putchar` implementation found |

Current receive/send method: hardware is configured for DMA-capable UART, but current application code does not start polling, interrupt, DMA, or DMA+IDLE receive. No transmit call was found outside HAL driver code.

## 6. Interrupt & DMA Map

| Interrupt / DMA | Purpose | Priority | Current handler/callback | Risk |
|---|---|---|---|---|
| SysTick | HAL tick | preemption 15 in `.ioc` | `SysTick_Handler()` calls `HAL_IncTick()` | Low; normal HAL timebase |
| USART1_IRQn | USART1 global interrupt | 0/0 | `USART1_IRQHandler()` calls `HAL_UART_IRQHandler(&huart1)` | Medium; enabled but no receive startup/callback |
| DMA2_Stream2_IRQn | USART1 RX DMA | 0/0 | `DMA2_Stream2_IRQHandler()` calls `HAL_DMA_IRQHandler(&hdma_usart1_rx)` | Medium; enabled but no RX buffer/start |
| DMA2_Stream7_IRQn | USART1 TX DMA | 0/0 | `DMA2_Stream7_IRQHandler()` calls `HAL_DMA_IRQHandler(&hdma_usart1_tx)` | Low/Medium; enabled but no TX use |
| TIM5_IRQn | Periodic timer candidate | 0/0 | `TIM5_IRQHandler()` calls `HAL_TIM_IRQHandler(&htim5)` | Medium; IRQ enabled but timer not started and no callback |
| EXTI4_IRQn | PA4 `Data_Ready_INT` | not configured | no `EXTI4_IRQHandler` found | High; EXTI pin configured but interrupt not functional |
| SPI1 IRQ | SPI1 | not configured | none | Design choice if polling/blocking transfer intended later |
| TIM1/TIM3 IRQ/DMA | PWM timers | not configured | none | Normal for PWM if no interrupts needed |

DMA details:

| DMA | Request | Direction | Mode | Priority | Evidence |
|---|---|---|---|---|---|
| DMA2 Stream2 Channel4 | USART1_RX | peripheral-to-memory | normal | low | `.ioc:5-17`, `Core/Src/usart.c:87-102` |
| DMA2 Stream7 Channel4 | USART1_TX | memory-to-peripheral | normal | low | `.ioc:18-27`, `Core/Src/usart.c:105-120` |

Real-time behavior scan:

| Item | Result |
|---|---|
| `HAL_Delay` in application | Not found |
| ISR long blocking code | Not found in user sections |
| ISR heavy computation | Not found |
| ISR UART send | Not found |
| Periodic scheduler | TIM5 configured but not started; no callback |
| BMI088 EXTI | GPIO configured, NVIC/handler absent |

## 7. Current Software Execution Flow

Current program startup call chain:

```text
Reset
 -> startup_stm32f405xx.s
 -> main()
 -> HAL_Init()
 -> SystemClock_Config()
 -> MX_GPIO_Init()
 -> MX_DMA_Init()
 -> MX_USART1_UART_Init()
 -> MX_SPI1_Init()
 -> MX_TIM1_Init()
 -> MX_TIM3_Init()
 -> MX_TIM5_Init()
 -> while(1)
```

Evidence: `Core/Src/main.c:78-98`.

Current `while(1)` call chain:

```text
while(1)
 -> no application code
```

Evidence: `Core/Src/main.c:105-110`.

Interrupt call chain currently present:

```text
SysTick_Handler()
 -> HAL_IncTick()

USART1_IRQHandler()
 -> HAL_UART_IRQHandler(&huart1)

TIM5_IRQHandler()
 -> HAL_TIM_IRQHandler(&htim5)

DMA2_Stream2_IRQHandler()
 -> HAL_DMA_IRQHandler(&hdma_usart1_rx)

DMA2_Stream7_IRQHandler()
 -> HAL_DMA_IRQHandler(&hdma_usart1_tx)
```

Evidence: `Core/Src/stm32f4xx_it.c:186-254`.

## 8. Existing Software Modules

| Module | Current status | Evidence / reason |
|---|---|---|
| SPI Driver | 已经有骨架 | CubeMX generated `SPI1` init in `Core/Src/spi.c:40-52`; no user wrapper or transactions. |
| UART Driver | 已经有骨架 | CubeMX generated USART1 + DMA in `Core/Src/usart.c:43-124`; no receive/transmit start logic. |
| PWM Driver | 已经有骨架 | TIM1/TIM3 PWM init exists in `Core/Src/tim.c:47-164`; no PWM start or abstraction. |
| Time / Scheduler | 已经有骨架 | TIM5 init and IRQ exist in `Core/Src/tim.c:181-243`; no start/callback/scheduler. |
| BMI088 Driver | 完全不存在 | Only GPIO labels and SPI1 exist; no BMI088 register/driver code. |
| BMI088 Heater | 已经有骨架 | TIM3 CH3 PWM exists; no heater control, target, PID, or start. |
| Servo | 已经有骨架 | TIM1 CH1~CH4 PWM exists; no servo mapping, pulse API, range, or start. |
| Host Communication | 已经有骨架 | USART1 hardware exists; no A133P protocol/buffer/parser. |
| CRC | 完全不存在 | HAL CRC module disabled in `Core/Inc/stm32f4xx_hal_conf.h:43`; SPI CRC disabled in `Core/Src/spi.c:50`. |
| PID | 完全不存在 | No PID code found in application files. |
| Filter | 完全不存在 | No application filter code found. CMSIS DSP contains generic vendor library code only. |
| Attitude Estimator | 完全不存在 | No attitude estimator code found. |
| Quaternion | 完全不存在 | No application quaternion code found. CMSIS DSP has generic vendor sources only. |
| X-Tail Mixer | 完全不存在 | No mixer/control allocation code found. |
| Flight Controller | 完全不存在 | No flight controller module found. |
| Flight State Machine | 完全不存在 | No state machine code found. |
| Failsafe | 完全不存在 | No failsafe logic found. |
| Debug / Telemetry | 完全不存在 | No printf redirect, UART telemetry, SWO/EventRecorder application code found. |

## 9. C/C++ Status

| Item | Current status | Evidence |
|---|---|---|
| Keil compiler | ARMCC 5.06 update 6 build 750 | `MDK-ARM/A133P_Dart.uvprojx:13` |
| Arm Compiler 6 | disabled | `MDK-ARM/A133P_Dart.uvprojx:14` |
| Global C++ compiler use | disabled / C mode | `MDK-ARM/A133P_Dart.uvprojx:96` has `UseCPPCompiler=0` |
| `.cpp/.hpp` files | none found in application/project tree | `rg --files .. -g "*.cpp" ...` returned no project C++ files |
| C/C++ linkage guards | present in CubeMX headers | `Core/Inc/main.h`, `spi.h`, `tim.h`, `usart.h` have `extern "C"` blocks |
| C++ standard | not actively used | Keil project has language fields, but AC6 is disabled and no C++ source is present |
| RTTI | not enabled in global AC6 fields | `MDK-ARM/A133P_Dart.uvprojx:338`; currently no C++ build to validate |
| Exceptions | unable to confirm from current active AC5 C-only build | No C++ source or explicit C++ exception option observed |
| Global constructors/startup risk | currently not active | No C++ source/global objects; would need review before Stage 1 C++ introduction |

Conclusion: The project is currently a C/HAL CubeMX project. It has enough `extern "C"` guards in generated headers for future mixed C/C++ use, but Keil is not currently compiling any user `.cpp` files and no C-to-C++ bridge exists.

## 10. Conflicts & Risks

### Critical

None confirmed from current static project files. No pin or DMA stream conflict was confirmed.

### High

| Risk | Reason | Evidence |
|---|---|---|
| PA4 EXTI is non-functional as currently generated | GPIO is configured as rising-edge EXTI, but NVIC/IRQ handler is absent. BMI088 data-ready cannot be serviced. | `.ioc:101-104`, `Core/Src/gpio.c:63-66`, no `EXTI4_IRQHandler` in `stm32f4xx_it.c` |
| PWM outputs are not active | TIM1/TIM3 PWM initialization exists but no `HAL_TIM_PWM_Start` call exists. | `Core/Src/tim.c:63,143`; no `HAL_TIM_PWM_Start` match |
| A133P UART communication is not actually running | USART1 DMA/IRQ are configured, but no RX/TX start, buffer, callback, IDLE handling, or parser exists. | `Core/Src/usart.c:43-124`, `Core/Src/main.c:99-110` |

### Medium

| Risk | Reason | Evidence |
|---|---|---|
| TIM1 current PWM frequency may not match servo requirements | Current theoretical frequency is 2563.48 Hz with ARR=65535/PSC=0. User explicitly said not to assume servo requirements, so this is marked as risk needing confirmation. | `Core/Src/tim.c:48-50`; APB2 timer clock `.ioc:161` |
| TIM3 heater PWM frequency and GPIO speed may need confirmation | Heater PWM is 1281.74 Hz, GPIO speed LOW. It may be acceptable, but no heater target/control code exists. | `Core/Src/tim.c:129-155`, `Core/Src/tim.c:286-290` |
| TIM5 scheduler candidate is not started and frequency may be unintentional | If started, update rate is about 496.5 Hz because PSC/ARR both include +1 in timer formula. | `Core/Src/tim.c:182-184`; no `HAL_TIM_Base_Start_IT` |
| All enabled peripheral IRQs use priority 0/0 except SysTick | USART1, DMA streams, TIM5 all priority 0/0. This may be a deliberate early default, but real-time priority policy is not designed. | `.ioc:73-85`, `Core/Src/dma.c:47-51`, `Core/Src/usart.c:123` |
| Clock source mismatch needs board-level confirmation | HSE pins are configured, but generated system clock uses HSI PLL source. | `.ioc:118-121`; `Core/Src/main.c:131-139` |

### Low

| Risk | Reason | Evidence |
|---|---|---|
| Build artifacts are present in project tree | `MDK-ARM/A133P_Dart/` contains `.o/.d/.crf/.axf/.hex/.map` outputs. | `rg --files ..` listing |
| `.cmsis/` contains large generic ARM device/example cache | Not referenced as application code; may clutter searches and version control. | `.cmsis` file listing |
| No `User/` layer exists yet | This is expected for Stage 0, but Stage 1 should establish the user-owned boundary clearly. | `Get-ChildItem ..` shows only `Core`, `Drivers`, `MDK-ARM`, `.ioc`, `.mxproject` |

## 11. Unknown / Need Confirmation

| Topic | Status |
|---|---|
| Servo1~4 physical mapping to four X-tail surfaces | 无法从当前工程确认 |
| TIM1 PWM target frequency and pulse width range | 尚未确定 |
| TIM3 heater target temperature, control period, duty limit | 尚未确定 |
| BMI088 exact interrupt mapping: ACC INT vs GYRO INT | 无法从当前工程确认 |
| Whether PA4 `Data_Ready_INT` is wired to BMI088 accel, gyro, or shared logic | 无法从当前工程确认 |
| BMI088 SPI mode/frequency against actual board and sensor datasheet requirement | 需要 Stage 1 前确认 |
| A133P serial protocol, frame format, CRC type | 待后续 Stage 设计 |
| UART RX buffer size and overflow policy | 不存在，待后续 Stage 设计 |
| UART IDLE Line strategy | 不存在，待后续 Stage 设计 |
| Control loop rate and scheduler model | 待后续 Stage 设计 |
| Attitude algorithm selection | 待后续 Stage 设计 |
| PID structure and parameters | 待后续 Stage 设计 |
| X-tail mixer matrix | 待后续 Stage 设计 |
| Failsafe behavior | 待后续 Stage 设计 |
| Whether HSE should be used as real system clock source | 需要硬件/时钟设计确认 |
| Whether EIDE `.cmsis` cache and build artifacts should be committed | 需要仓库管理策略确认 |

## 12. Recommendations for Stage 1

Stage 1 should stay at architecture/design level first, not implementation. Recommended focus:

| Area | Stage 1 decision needed |
|---|---|
| Source ownership boundary | Define `Core/` as CubeMX-owned and create a small `User/` ownership model only after agreeing directory layout. |
| Hardware config baseline | Decide whether `.ioc` clock source should use HSI or HSE; fix only after explicit Stage 1 approval. |
| PWM abstraction | Confirm servo frequency, pulse range, polarity, safe startup value, and channel-to-surface mapping before writing any servo class. |
| BMI088 interface | Confirm SPI mode/speed, CS polarity, ACC/GYRO interrupt wiring, startup sequence, and whether EXTI is required. |
| Heater control | Treat BMI088 measurement and heater control as separate responsibilities; define target temperature and duty limits before implementation. |
| Host communication | Define A133P protocol, DMA+IDLE or other receive model, buffer ownership, overflow policy, CRC, timeout, and failsafe trigger conditions. |
| Scheduler | Decide whether TIM5 is the scheduler timer, what control frequency is required, and which work is allowed in ISR vs main/task context. |
| C++ integration | Add C++ only with an explicit mixed-language build plan: static objects, no dynamic allocation, no exceptions/RTTI unless deliberately enabled. |
| Verification | Establish minimal hardware bring-up tests for SPI ID read, UART loop/host test, PWM output frequency, and EXTI event capture before building flight logic. |

