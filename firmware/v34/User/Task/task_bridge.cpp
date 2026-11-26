#include "task_bridge.h"
#include <string.h>

#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#include "Application/app_dart_system.h"
#include "Common/common_types.h"
#include "Common/control_command.h"
#include "Config/control_config.h"
#include "Config/imu_heater_config.h"
#include "Config/servo_config.h"
#include "Driver/drv_gpio_pin.h"
#include "Driver/drv_pwm_channel.h"
#include "Driver/drv_spi_bus.h"
#include "Driver/drv_timebase.h"
#include "Driver/drv_uart_dma.h"
#include "Device/dev_bmi088.h"
#include "Device/dev_host_serial.h"
#include "Config/dart_light_config.h"
#include "Driver/drv_uart_half_duplex.h"
#include "Device/dev_dart_light.h"
#include "Device/dev_key_long_press.h"

static Class_SpiBus bmi088_spi;
static Class_GpioPin bmi088_acc_cs;
static Class_GpioPin bmi088_gyro_cs;
static Class_PwmChannel tim1_pwm_ch1;
static Class_PwmChannel tim1_pwm_ch2;
static Class_PwmChannel tim1_pwm_ch3;
static Class_PwmChannel tim1_pwm_ch4;
static Class_PwmChannel imu_heater_pwm;
static Class_UartDma host_uart;
static Class_HostSerial host_serial;// 上位机串口设备 对象
static Class_Timebase system_timebase;
static Class_DartSystem dart_system;
static Class_UartHalfDuplex dart_light_uart;
static Class_DartLight dart_light;
static Class_GpioPin dart_light_key_pin;
static Class_GpioPin camera_trigger_pin;
static Class_KeyLongPress dart_light_key;
static uint8_t dart_light_rx_ring[32];

volatile ResultCode dart_light_init_result = ResultCode::NotReady;
volatile ResultCode dart_light_poll_result = ResultCode::NotReady;
volatile DartLightTeam dart_light_current_team = kDartLightDefaultTeam;
volatile ResultCode dart_light_set_team_result = ResultCode::NotReady;
volatile uint8_t dart_light_key_held_low = 0U;
volatile uint32_t dart_light_key_toggle_count = 0U;
DartLightStatus dart_light_status_snapshot = {};
static uint32_t dart_light_last_set_team_ms = 0U;
static uint32_t dart_light_last_status_ms = 0U;

volatile uint8_t imu_telemetry_mode = kTaskImuTelemetryHostProtocol;

static uint8_t host_rx_dma_buffer[256];
static volatile ResultCode host_uart_rx_result = ResultCode::NotReady;

volatile uint16_t test_PulseWidthUs1 = 0U;
volatile uint16_t test_PulseWidthUs2 = 0U;
volatile uint16_t test_PulseWidthUs3 = 0U;
volatile uint16_t test_PulseWidthUs4 = 0U;

//测试BMI088
static const uint8_t kBmi088BringupMaxAttempts = 5U;
volatile ResultCode bmi088_probe_result = ResultCode::NotReady;
volatile uint8_t bmi088_acc_chip_id = 0U;
volatile uint8_t bmi088_gyro_chip_id = 0U;
volatile ResultCode bmi088_probe_acc_activation_result = ResultCode::NotReady;
volatile ResultCode bmi088_probe_acc_id_result = ResultCode::NotReady;
volatile ResultCode bmi088_probe_gyro_id_result = ResultCode::NotReady;
volatile uint8_t bmi088_probe_acc_activation_rx[3] = {0U};
volatile uint8_t bmi088_probe_acc_id_rx[3] = {0U};
volatile uint8_t bmi088_probe_gyro_id_rx[2] = {0U};
volatile uint8_t bmi088_bringup_attempt_count = 0U;
volatile ResultCode bmi088_init_attempt_results[5] = {
    ResultCode::NotReady, ResultCode::NotReady, ResultCode::NotReady,
    ResultCode::NotReady, ResultCode::NotReady
};
volatile ResultCode bmi088_probe_attempt_results[5] = {
    ResultCode::NotReady, ResultCode::NotReady, ResultCode::NotReady,
    ResultCode::NotReady, ResultCode::NotReady
};
volatile uint8_t bmi088_acc_chip_id_attempts[5] = {0U};
volatile uint8_t bmi088_gyro_chip_id_attempts[5] = {0U};
volatile ResultCode bmi088_init_result = ResultCode::NotReady;
volatile ResultCode bmi088_sample_result = ResultCode::NotReady;
volatile Bmi088RawSample bmi088_raw_sample = {};
ImuData bmi088_body_imu_data = {};
volatile uint32_t bmi088_sample_count = 0U;

// 测试用的过载指令
HostOverloadCommand overload_cmd;

static ResultCode task_init_result = ResultCode::Ok;
static volatile ResultCode servo_pwm_bringup_result = ResultCode::NotReady;
static volatile uint8_t servo_pwm_failed_step = 0U;
static volatile ResultCode timebase_start_result = ResultCode::NotReady;
static volatile ResultCode control_output_result = ResultCode::NotReady;
static volatile ResultCode host_serial_poll_result = ResultCode::NotReady;
static volatile ResultCode host_overload_take_result = ResultCode::NotReady;
static volatile ResultCode imu_tx_result = ResultCode::NotReady;

static uint32_t last_processed_ms = 0U;
static uint8_t bmi088_poll_divider = 0U;
static uint64_t last_heater_update_us = 0ULL;
static bool has_heater_update = false;
static volatile ResultCode imu_heater_update_result = ResultCode::NotReady;

// 相机硬触发状态：100 Hz（每 10 帧 IMU 触发一次），图像序列号随触发递增。
static uint16_t camera_image_seq = 0U;
static bool camera_trigger_pulse_active = false;

// 闭环运行状态：主机指令先锁存为 n_y_c/n_z_c；只有新的、仍新鲜的 IMU 样本
// 才能推进控制器。成功解算的指令被保留到下一次有效更新；命令/IMU 超时、
// 初始化/PWM 未就绪或闭环解算失败时清空。时间戳均采用 TIM5 硬件 1 us 分辨率。
static float n_y_c = 0.0f;
static float n_z_c = 0.0f;
static bool host_command_valid = false;
static uint64_t last_host_command_us = 0ULL;
static uint32_t last_control_sample_seq = 0U;
static uint64_t last_control_timestamp_us = 0ULL;
static bool has_control_sample = false;
static BodyControlCommand last_closed_loop_command;

// static BodyControlCommand test_command;

static void DelayBmi088Ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

static void StoreBmi088RawSample(const Bmi088RawSample& sample)
{
    bmi088_raw_sample.acc_x = sample.acc_x;
    bmi088_raw_sample.acc_y = sample.acc_y;
    bmi088_raw_sample.acc_z = sample.acc_z;
    bmi088_raw_sample.gyro_x = sample.gyro_x;
    bmi088_raw_sample.gyro_y = sample.gyro_y;
    bmi088_raw_sample.gyro_z = sample.gyro_z;
    bmi088_raw_sample.temperature_raw = sample.temperature_raw;
    bmi088_raw_sample.temperature_deg_c = sample.temperature_deg_c;
}

static void StoreBmi088BodyImuData(const ImuData& sample)
{
    // 这里保存的是已经完成唯一坐标变换的机体坐标系数据；逐字段写入，
    // 发送路径随后只消费这份持久数据，避免原始坐标系数据再次流出。
    bmi088_body_imu_data.specific_force_x_mps2 = sample.specific_force_x_mps2;
    bmi088_body_imu_data.specific_force_y_mps2 = sample.specific_force_y_mps2;
    bmi088_body_imu_data.specific_force_z_mps2 = sample.specific_force_z_mps2;
    bmi088_body_imu_data.gyro_x_radps = sample.gyro_x_radps;
    bmi088_body_imu_data.gyro_y_radps = sample.gyro_y_radps;
    bmi088_body_imu_data.gyro_z_radps = sample.gyro_z_radps;
    bmi088_body_imu_data.temperature_deg_c = sample.temperature_deg_c;
    bmi088_body_imu_data.sample_timestamp_ms = sample.sample_timestamp_ms;
    bmi088_body_imu_data.sample_timestamp_us = sample.sample_timestamp_us;
    bmi088_body_imu_data.data_ready_seq = sample.data_ready_seq;
    bmi088_body_imu_data.valid = sample.valid;
}

static void StoreFirstTaskInitError(ResultCode result)
{
    if (task_init_result == ResultCode::Ok && result != ResultCode::Ok) {
        task_init_result = result;
    }
}

static bool CheckServoPwmStep(ResultCode result, uint8_t failed_step)
{
    if (result != ResultCode::Ok) {
        servo_pwm_bringup_result = result;
        servo_pwm_failed_step = failed_step;
        StoreFirstTaskInitError(result);
        return false;
    }

    return true;
}
/**
 * @brief 飞控任务初始化
 *
 * 完成各驱动、传感器、舵机及控制算法的初始化。
 */
void Task_Init(void)
{
    task_init_result = ResultCode::Ok;                  // 总初始化状态
    servo_pwm_bringup_result = ResultCode::NotReady;    // 舵机 PWM 上电结果
    servo_pwm_failed_step = 0U;                         // 舵机失败步骤号
    host_uart_rx_result = ResultCode::NotReady;         // 上位机串口接收结果
    timebase_start_result = ResultCode::NotReady;       // 时基启动结果
    control_output_result = ResultCode::NotReady;       // 控制输出结果
    host_serial_poll_result = ResultCode::NotReady;     // 上位机协议轮询结果
    host_overload_take_result = ResultCode::NotReady;   // 上位机过载接管结果
    imu_tx_result = ResultCode::NotReady;               // IMU 发送结果
    bmi088_probe_result = ResultCode::NotReady;         // BMI088 探测结果
    bmi088_acc_chip_id = 0U;                            // 加速度计芯片 ID
    bmi088_gyro_chip_id = 0U;                           // 陀螺仪芯片 ID
    bmi088_probe_acc_activation_result = ResultCode::NotReady; // 加速度计使能探测结果
    bmi088_probe_acc_id_result = ResultCode::NotReady;  // 加速度计 ID 探测结果
    bmi088_probe_gyro_id_result = ResultCode::NotReady; // 陀螺仪 ID 探测结果
    memset((void*)bmi088_probe_acc_activation_rx, 0, sizeof(bmi088_probe_acc_activation_rx)); // 清空探测原始字节
    memset((void*)bmi088_probe_acc_id_rx, 0, sizeof(bmi088_probe_acc_id_rx));
    memset((void*)bmi088_probe_gyro_id_rx, 0, sizeof(bmi088_probe_gyro_id_rx));
    bmi088_bringup_attempt_count = 0U;                  // BMI088 上电尝试次数
    for (uint8_t index = 0U; index < kBmi088BringupMaxAttempts; ++index) {
        bmi088_init_attempt_results[index] = ResultCode::NotReady;
        bmi088_probe_attempt_results[index] = ResultCode::NotReady;
        bmi088_acc_chip_id_attempts[index] = 0U;
        bmi088_gyro_chip_id_attempts[index] = 0U;
    }
    bmi088_init_result = ResultCode::NotReady;          // BMI088 初始化结果
    bmi088_sample_result = ResultCode::NotReady;        // BMI088 采样结果
    bmi088_raw_sample.acc_x = 0;                        // 原始采样数据清零
    bmi088_raw_sample.acc_y = 0;
    bmi088_raw_sample.acc_z = 0;
    bmi088_raw_sample.gyro_x = 0;
    bmi088_raw_sample.gyro_y = 0;
    bmi088_raw_sample.gyro_z = 0;
    bmi088_raw_sample.temperature_raw = 0;
    bmi088_raw_sample.temperature_deg_c = 0.0f;
    bmi088_body_imu_data = ImuData();                   // 机体 IMU 数据清零
    bmi088_sample_count = 0U;                           // 采样计数清零

    last_processed_ms = 0U;                             // 上次处理时间戳清零
    bmi088_poll_divider = 0U;                           // 采样分频计数
    last_heater_update_us = 0ULL;                       // 加热器上次更新微秒时间戳清零
    has_heater_update = false;                          // 加热器历史标记清零
    imu_heater_update_result = ResultCode::NotReady;    // 加热器更新结果复位

    // 闭环状态必须与控制器一起回到确定的初态：清除主机命令有效位和时间戳，
    // 丢弃历史 IMU 样本标识及最后有效输出，避免复位后沿用旧命令或旧 dt。
    // n_y_c/n_z_c 分别为主机 pitch/yaw 过载目标，初始为零。
    n_y_c = 0.0f;
    n_z_c = 0.0f;
    host_command_valid = false;
    last_host_command_us = 0ULL;
    last_control_sample_seq = 0U;
    last_control_timestamp_us = 0ULL;
    has_control_sample = false;
    last_closed_loop_command = BodyControlCommand();
    overload_cmd = HostOverloadCommand();
    dart_system.ResetClosedLoop();

    // 四路舵机置中位脉宽
    test_PulseWidthUs1 = kServoConfigs[0].neutral_pulse_us; 
    test_PulseWidthUs2 = kServoConfigs[1].neutral_pulse_us;
    test_PulseWidthUs3 = kServoConfigs[2].neutral_pulse_us;
    test_PulseWidthUs4 = kServoConfigs[3].neutral_pulse_us;

    StoreFirstTaskInitError(bmi088_spi.Bind(&hspi1));                                   // 绑定 BMI088 SPI
    StoreFirstTaskInitError(bmi088_acc_cs.Bind(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin));   // 加速度计片选
    StoreFirstTaskInitError(bmi088_gyro_cs.Bind(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin)); // 陀螺仪片选
    StoreFirstTaskInitError(camera_trigger_pin.Bind(CAMERA_TRIGGER_GPIO_Port, CAMERA_TRIGGER_Pin)); // 相机硬触发 PC3
    camera_trigger_pin.SetLow();                  // 初始为低电平
    camera_image_seq = 0U;                        // 图像序列号清零
    camera_trigger_pulse_active = false;          // 触发脉冲状态复位

    // 绑定 TIM1 的四路 PWM 通道（用于四路舵机，168 MHz / (167+1) = 1 MHz 计数基准）
    StoreFirstTaskInitError(tim1_pwm_ch1.Bind(&htim1, TIM_CHANNEL_1, 1000000U));
    StoreFirstTaskInitError(tim1_pwm_ch2.Bind(&htim1, TIM_CHANNEL_2, 1000000U));
    StoreFirstTaskInitError(tim1_pwm_ch3.Bind(&htim1, TIM_CHANNEL_3, 1000000U));
    StoreFirstTaskInitError(tim1_pwm_ch4.Bind(&htim1, TIM_CHANNEL_4, 1000000U));

    // 绑定 TIM3 的 PWM 通道（用于 IMU 加热器，APB1 定时器时钟 84 MHz）
    StoreFirstTaskInitError(imu_heater_pwm.Bind(&htim3, TIM_CHANNEL_3, 84000000U));

    // 绑定上位机串口和协议层
    StoreFirstTaskInitError(host_uart.Bind(&huart1));          // 上位机串口
    StoreFirstTaskInitError(host_serial.Bind(&host_uart));     // 上位机协议层

    // 绑定系统时基 TIM5（1 us 硬件计数分辨率 + 1 ms 溢出中断）
    StoreFirstTaskInitError(system_timebase.Bind(&htim5));     

    /*---------------------------------------------------------------*/

    // 绑定飞控系统对象
    if (task_init_result == ResultCode::Ok) {
        StoreFirstTaskInitError(dart_system.BindBmi088(&bmi088_spi, &bmi088_acc_cs, &bmi088_gyro_cs)); // 飞控绑定 IMU
        StoreFirstTaskInitError(dart_system.BindImuHeater(&imu_heater_pwm));                            // 绑定 IMU 加热器
        StoreFirstTaskInitError(dart_system.ConfigureImuHeater(MakeDefaultImuHeaterConfig()));          // 配置 45°C 温控
        StoreFirstTaskInitError(dart_system.StartImuHeater());                                          // 启动加热器 PWM（占空比 0）
        StoreFirstTaskInitError(dart_system.BindServo(&tim1_pwm_ch1, &kServoConfigs[0]));               // 绑定四路舵机
        StoreFirstTaskInitError(dart_system.BindServo(&tim1_pwm_ch2, &kServoConfigs[1]));
        StoreFirstTaskInitError(dart_system.BindServo(&tim1_pwm_ch3, &kServoConfigs[2]));
        StoreFirstTaskInitError(dart_system.BindServo(&tim1_pwm_ch4, &kServoConfigs[3]));
    }

    // BMI088 上电接口切换和陀螺仪复位存在时序窗口。执行有限次完整初始化，
    // 每次随后保存一笔真实 Probe，既验证最终状态，也保留最后一次原始字节。
    if (task_init_result == ResultCode::Ok) {
        for (uint8_t attempt = 0U; attempt < kBmi088BringupMaxAttempts; ++attempt) {
            bmi088_bringup_attempt_count = static_cast<uint8_t>(attempt + 1U);   // 记录当前尝试次数
            bmi088_init_result = dart_system.InitializeBmi088(DelayBmi088Ms);    // 执行一次完整初始化

            uint8_t acc_chip_id = 0U;
            uint8_t gyro_chip_id = 0U;
            Bmi088ProbeTrace probe_trace = {};
            bmi088_probe_result =
                dart_system.ProbeBmi088WithTrace(acc_chip_id, gyro_chip_id, probe_trace); // 探测并保存原始字节

            bmi088_init_attempt_results[attempt] = bmi088_init_result;          // 保存每次尝试结果
            bmi088_probe_attempt_results[attempt] = bmi088_probe_result;
            bmi088_acc_chip_id_attempts[attempt] = acc_chip_id;
            bmi088_gyro_chip_id_attempts[attempt] = gyro_chip_id;
            bmi088_acc_chip_id = acc_chip_id;                                   // 记录最终芯片 ID
            bmi088_gyro_chip_id = gyro_chip_id;
            bmi088_probe_acc_activation_result = probe_trace.acc_activation_result;
            bmi088_probe_acc_id_result = probe_trace.acc_id_result;
            bmi088_probe_gyro_id_result = probe_trace.gyro_id_result;
            for (uint8_t index = 0U; index < 3U; ++index) {
                bmi088_probe_acc_activation_rx[index] = probe_trace.acc_activation_rx[index];
                bmi088_probe_acc_id_rx[index] = probe_trace.acc_id_rx[index];
            }
            for (uint8_t index = 0U; index < 2U; ++index) {
                bmi088_probe_gyro_id_rx[index] = probe_trace.gyro_id_rx[index];
            }

            if (bmi088_init_result == ResultCode::Ok &&
                bmi088_probe_result == ResultCode::Ok) {
                break;                                                          // 初始化成功，退出重试
            }
            if (attempt + 1U < kBmi088BringupMaxAttempts) {
                DelayBmi088Ms(30U);                                             // 失败后延时再重试
            }
        }
    }

    /*至此，所有绑定和设备初始化完成*/

    /*开始启动:舵机、串口、系统时基*/

    if (task_init_result == ResultCode::Ok) {
        servo_pwm_bringup_result = ResultCode::Ok;

        // 四路舵机输出中位脉宽，逐步校验
        ResultCode servo_result = tim1_pwm_ch1.SetPulseWidthUs(test_PulseWidthUs1);
        if (!CheckServoPwmStep(servo_result, 1U)) {
            return;
        }

        servo_result = tim1_pwm_ch2.SetPulseWidthUs(test_PulseWidthUs2);
        if (!CheckServoPwmStep(servo_result, 2U)) {
            return;
        }

        servo_result = tim1_pwm_ch3.SetPulseWidthUs(test_PulseWidthUs3);
        if (!CheckServoPwmStep(servo_result, 3U)) {
            return;
        }

        servo_result = tim1_pwm_ch4.SetPulseWidthUs(test_PulseWidthUs4);
        if (!CheckServoPwmStep(servo_result, 4U)) {
            return;
        }

        // 启动四路舵机 PWM 输出
        servo_result = tim1_pwm_ch1.Start();
        if (!CheckServoPwmStep(servo_result, 5U)) {
            return;
        }

        servo_result = tim1_pwm_ch2.Start();
        if (!CheckServoPwmStep(servo_result, 6U)) {
            return;
        }

        servo_result = tim1_pwm_ch3.Start();
        if (!CheckServoPwmStep(servo_result, 7U)) {
            return;
        }

        servo_result = tim1_pwm_ch4.Start();
        if (!CheckServoPwmStep(servo_result, 8U)) {
            return;
        }
    }

    if (task_init_result == ResultCode::Ok) {
        const ResultCode rx_start_result =
            host_uart.StartReceiveCircular(host_rx_dma_buffer, sizeof(host_rx_dma_buffer)); // 开启上位机 DMA 接收
        host_uart_rx_result = rx_start_result;
        StoreFirstTaskInitError(rx_start_result);
    }

    if (task_init_result == ResultCode::Ok) {
        timebase_start_result = system_timebase.Start();   // 启动系统时基
        StoreFirstTaskInitError(timebase_start_result);
    }

    dart_light_init_result = ResultCode::NotReady;
    dart_light_poll_result = ResultCode::NotReady;
    dart_light_current_team = kDartLightDefaultTeam;
    dart_light_set_team_result = ResultCode::NotReady;
    dart_light_key_held_low = 0U;
    dart_light_key_toggle_count = 0U;
    dart_light_status_snapshot = DartLightStatus();
    dart_light_last_set_team_ms = 0U;
    dart_light_last_status_ms = 0U;

    // 绑定并初始化飞镖发光装置 UART4
    ResultCode light_bind_res = dart_light_uart.Bind(&huart4);
    if (light_bind_res == ResultCode::Ok) {
        light_bind_res = dart_light.Bind(&dart_light_uart);
    }
    if (light_bind_res == ResultCode::Ok) {
        light_bind_res = dart_light_uart.StartReceive(dart_light_rx_ring, sizeof(dart_light_rx_ring));
    }
    if (light_bind_res == ResultCode::Ok) {
        light_bind_res = dart_light.RequestSetTeam(kDartLightDefaultTeam);
    }
    dart_light_init_result = light_bind_res;

    // 绑定 PC13 快捷切色按键
    if (dart_light_key_pin.Bind(KEY_GPIO_Port, KEY_Pin) == ResultCode::Ok) {
        dart_light_key.Bind(
            &dart_light_key_pin,
            kDartLightKeyDebounceMs,
            kDartLightKeyHoldMs);
    }
}

int number_of_camera_triggers = 0;
/**
 * @brief 飞控任务主循环
 *
 * 由主程序周期调用。每个毫秒刻度依次完成：锁存最新主机过载命令、按 1 ms（1000 Hz）
 * 读取 IMU 并异步发送遥测、按 kHostCommandTimeoutMs 与 kImuStaleTimeoutMs（微秒级精度）判定新鲜度和
 * 陈旧门控；只有新 IMU 样本满足全部门控时才基于 1 us 硬件时间戳差推进闭环。
 * 成功结果锁存为最后有效命令，下一次更新前持续使用；超时、未就绪或解算失败则
 * 复位闭环并发送空指令，使舵机进入回中路径。
 */
void Task_Loop(void)
{
    // 每个毫秒刻度只处理一次，避免重复执行
    const uint32_t now_ms = system_timebase.NowMs();
    if (now_ms == last_processed_ms) {
        return;
    }
    last_processed_ms = now_ms;

    // 相机触发脉冲：上一拍拉高后本拍拉低，形成约 1 ms 的高电平冲激。
    if (camera_trigger_pulse_active) {
        camera_trigger_pin.SetLow();
        camera_trigger_pulse_active = false;
        number_of_camera_triggers++;
    }

    // 轮询并锁存最近一条有效主机命令：pitch_overload_g 映射到 n_y_c，
    // yaw_overload_g 映射到 n_z_c；微秒时间戳只在成功锁存时刷新。
    host_serial_poll_result = host_serial.Poll();
    HostOverloadCommand latest_overload;
    host_overload_take_result = Task_TakeLatestHostOverload(latest_overload);
    if (host_overload_take_result == ResultCode::Ok && latest_overload.valid) {
        overload_cmd = latest_overload;
        n_y_c = latest_overload.pitch_overload_g;
        n_z_c = latest_overload.yaw_overload_g;
        last_host_command_us = system_timebase.NowUs();
        host_command_valid = true;
    }

    bool new_imu_sample = false;

    // output_test();
    // control_output_result = dart_system.ApplyBodyControlCommand(test_command);

    // 每 1 个毫秒刻度采样一次 IMU（1000 Hz）并通过 DMA 异步发送遥测，同时记录 TIM5 硬件 1 us 时间戳。
    // 样本序号或微秒时间戳变化才标记为新控制样本，避免对同一已标记样本重复解算。
    ++bmi088_poll_divider;
    if (bmi088_poll_divider >= 1U) {
        bmi088_poll_divider = 0U;
        if (bmi088_init_result == ResultCode::Ok) {
            Bmi088RawSample candidate = {};
            bmi088_sample_result = dart_system.ReadBmi088RawSample(candidate);
            if (bmi088_sample_result == ResultCode::Ok) {
                StoreBmi088RawSample(candidate);
                ++bmi088_sample_count;
                const uint64_t sample_timestamp_us = system_timebase.NowUs();
                const uint32_t sample_timestamp_ms =
                    static_cast<uint32_t>(sample_timestamp_us / 1000ULL);
                const ImuData imu_candidate = Bmi088RawSampleToImuData(
                    candidate,
                    sample_timestamp_ms,
                    sample_timestamp_us,
                    bmi088_sample_count);
                StoreBmi088BodyImuData(imu_candidate);
                // 判断是否为一次新的控制采样
                new_imu_sample =
                    !has_control_sample ||
                    imu_candidate.sample_timestamp_us != last_control_timestamp_us ||
                    imu_candidate.data_ready_seq != last_control_sample_seq;
                // 相机硬触发：已发射且每 8 帧 IMU（1000 Hz / 10 = 100 Hz）触发一次，
                // PC3 拉高形成冲激，图像序列号递增并随本帧上报。
                uint8_t trigger_flag = 0U;
                if (overload_cmd.is_launched != 0U && (bmi088_sample_count % 10U) == 0U) {
                //  if (bmi088_sample_count % 100U == 0U) {
                    camera_trigger_pin.SetHigh();
                    camera_trigger_pulse_active = true;
                    ++camera_image_seq;
                    trigger_flag = 1U;
                }
                const ClosedLoopControlOutput& last_output = dart_system.GetLastClosedLoopOutput();
                imu_tx_result = Task_SendImuData(
                    bmi088_body_imu_data, last_closed_loop_command,
                    last_output.omega_y_c, last_output.omega_z_c,
                    trigger_flag, camera_image_seq);
            }
        }
    }

    // IMU 加热器温度闭环：独立于飞控闭环，按 kImuHeaterUpdatePeriodMs 周期更新，
    // 采用 1 us 硬件时间差计算真实 dt_s，不因主机命令/IMU 陈旧而停摆。
    if (task_init_result == ResultCode::Ok && dart_system.IsImuHeaterBound()) {
        const uint64_t heater_now_us = system_timebase.NowUs();
        const uint64_t heater_period_us =
            static_cast<uint64_t>(kImuHeaterUpdatePeriodMs) * 1000ULL;
        if (!has_heater_update ||
            heater_now_us - last_heater_update_us >= heater_period_us) {
            if (bmi088_body_imu_data.valid) {
                const float heater_dt_s = has_heater_update
                    ? static_cast<float>(heater_now_us - last_heater_update_us) * 1.0e-6f
                    : static_cast<float>(kImuHeaterUpdatePeriodMs) * 0.001f;
                imu_heater_update_result = dart_system.UpdateImuHeater(
                    bmi088_body_imu_data.temperature_deg_c,
                    heater_dt_s);
            }
            last_heater_update_us = heater_now_us;
            has_heater_update = true;
        }
    }

    // 主机命令与 IMU 分别按 kHostCommandTimeoutMs、kImuStaleTimeoutMs（折算为微秒）保鲜；
    // 二者任一陈旧即禁止闭环输出。
    const uint64_t check_now_us = system_timebase.NowUs();
    const bool host_fresh = host_command_valid &&
        (check_now_us - last_host_command_us <=
         static_cast<uint64_t>(kHostCommandTimeoutMs) * 1000ULL);
    const bool imu_fresh = bmi088_body_imu_data.valid &&
        (check_now_us - bmi088_body_imu_data.sample_timestamp_us <=
         static_cast<uint64_t>(kImuStaleTimeoutMs) * 1000ULL);

    // 仅新的 IMU 样本、两路数据新鲜且初始化/PWM 就绪时才推进一次闭环控制解算。
    if (new_imu_sample && host_fresh && imu_fresh &&
        task_init_result == ResultCode::Ok &&
        servo_pwm_bringup_result == ResultCode::Ok) {
        // 首帧采用名义 1 ms（1000 Hz）；后续 dt 来自相邻两个已用于控制的 IMU 硬件 1 us 时间戳差，
        // 消除毫秒整数量化误差，同时不会把主循环调用间隔或重复样本误作控制周期。
        float dt_s = 0.001f;
        if (has_control_sample) {
            const uint64_t elapsed_us =
                bmi088_body_imu_data.sample_timestamp_us - last_control_timestamp_us;
            if (elapsed_us > 0ULL) {
                dt_s = static_cast<float>(elapsed_us) * 1.0e-6f;
            }
        }

        // 控制逻辑：
        // 1. 判断已经发射时，只控制 roll 闭环；
        // 2. 当判断到已经识别绿灯时，再开始控制 pitch 和 yaw 闭环；
        // 3. 未发射时，三轴均不控制（中位安全回中）。
        const bool is_launched = (overload_cmd.is_launched != 0U);
        const bool is_green_detected = (overload_cmd.is_base_green_detected != 0U);
        const bool enable_roll = is_launched;
        const bool enable_pitch_yaw = is_launched && is_green_detected;

        BodyControlCommand candidate_command;// 用于存储计算出的控制指令
        control_output_result = dart_system.UpdateClosedLoop(
            n_y_c,
            n_z_c,
            bmi088_body_imu_data,
            dt_s,
            candidate_command,
            enable_roll,
            enable_pitch_yaw);
        if (control_output_result == ResultCode::Ok && candidate_command.valid) {
            // 只锁存完整成功的输出；在下一个有效新样本到来前继续使用该最后有效命令。
            last_closed_loop_command = candidate_command;
            last_control_sample_seq = bmi088_body_imu_data.data_ready_seq;
            last_control_timestamp_us = bmi088_body_imu_data.sample_timestamp_us;
            has_control_sample = true;
        } else {
            // 解算失败不保留旧输出：复位积分等内部状态并清空锁存命令，随后回中。
            dart_system.ResetClosedLoop();
            last_closed_loop_command = BodyControlCommand();
            has_control_sample = false;
        }
    } else if (!host_fresh || !imu_fresh ||
               task_init_result != ResultCode::Ok ||
               servo_pwm_bringup_result != ResultCode::Ok) {
        // 命令/IMU 陈旧或初始化未就绪时，复位控制器并丢弃锁存命令，阻止旧输出维持。
        dart_system.ResetClosedLoop();
        last_closed_loop_command = BodyControlCommand();
        has_control_sample = false;
        control_output_result = host_fresh && imu_fresh ?
            ResultCode::NotReady : ResultCode::Timeout;
    }

    // 最后一道门控：仅在全部条件仍有效时输出最后有效命令；否则改为空指令，
    // 由下层按中位安全位置执行回中。
    BodyControlCommand command = last_closed_loop_command;
    command.valid = command.valid && host_fresh && imu_fresh &&
        task_init_result == ResultCode::Ok &&
        servo_pwm_bringup_result == ResultCode::Ok;
    if (!command.valid) {
        command = BodyControlCommand();
    }

    // 应用机体控制指令
    control_output_result = dart_system.ApplyBodyControlCommand(command);

    // 飞镖发光装置按键快捷切色检测（PC13 低电平保持 5 s 触发切换）
    if (dart_light_key.Update(now_ms)) {
        ++dart_light_key_toggle_count;
        dart_light_current_team = (dart_light_current_team == DartLightTeam::Red)
            ? DartLightTeam::Blue
            : DartLightTeam::Red;
        dart_light_set_team_result = dart_light.RequestSetTeam(dart_light_current_team);
    }
    dart_light_key_held_low = dart_light_key.IsHeldLow() ? 1U : 0U;

    // 方案二：每 2 s 周期重发改色指令，保证装置在自身 5V 上电后 3 s 窗口内一定能收到
    if (now_ms - dart_light_last_set_team_ms >= kDartLightSetTeamResendMs) {
        dart_light_last_set_team_ms = now_ms;
        dart_light.RequestSetTeam(dart_light_current_team);
    }

    // 周期查询装置状态（实际颜色/温度/故障），用于确认改色结果与诊断
    if (now_ms - dart_light_last_status_ms >= kDartLightStatusQueryMs) {
        dart_light_last_status_ms = now_ms;
        dart_light.RequestStatus();
    }

    // 飞镖发光装置通信状态机轮询
    dart_light_poll_result = dart_light.Poll(system_timebase.NowUs());
    dart_light_set_team_result = dart_light.GetLastSetTeamResult();
    dart_light_status_snapshot = dart_light.GetStatus();
}
/**
 * @brief 取走最新一条从 USART1 解码出的有效物理过载指令。
 * @param command 接收最新指令的目标。
 * @return 有可用的最新指令时返回 ResultCode::Ok；否则返回 ResultCode::NotReady。
 */
ResultCode Task_TakeLatestHostOverload(HostOverloadCommand& command)
{
    return host_serial.TakeLatestOverload(command);
}

/**
 * @brief 将一份有效的六轴 IMU 采样编码并通过 USART1 启动一次异步 DMA 发送。
 * @param imu 要发送的有效 IMU 采样。
 * @return 编码并成功启动发送时返回 ResultCode::Ok；采样无效或编码失败时返回
 *         ResultCode::InvalidData；串口未绑定时返回 ResultCode::ConfigError。
 */
ResultCode Task_SendImuData(const ImuData& imu, const BodyControlCommand& command,
                            float overload_yaw_rate_radps, float overload_pitch_rate_radps,
                            uint8_t trigger_flag, uint16_t image_seq)
{
    if (imu_telemetry_mode == kTaskImuTelemetryVofaJustFloat) //vofa调试用
    {
        return Task_SendVofaImuData(imu);
    }
    return host_serial.SendImuData(imu, command, overload_yaw_rate_radps, overload_pitch_rate_radps,
                                   trigger_flag, image_seq);
}

ResultCode Task_SendVofaImuData(const ImuData& imu)
{
    return host_serial.SendVofaJustFloatData(imu);
}

// void output_test()
// {
//     //当前测试逻辑为：当任意轴过载指令不为零时，测试姿态角 yaw指令 置为 0.9
//     if(overload_cmd.yaw_overload_g != 0.0f || overload_cmd.pitch_overload_g != 0.0f || overload_cmd.is_launched != 0U)
//     {
        
//         test_command.valid = true;
//         test_command.yaw = 0.56f; 
//         test_command.roll = 0.0f;
//         test_command.pitch= 0.0f;
//     }
//     // else
//     // {
//     //     test_command.valid = false;
//     //     test_command.pitch = 0.0f;
//     //     test_command.roll = 0.0f;
//     //     test_command.yaw= 0.0f;

//     // }
// }

//------------------------------------- STM32 HAL 回调函数 -------------------------------------
ResultCode Task_DartLightSetTeam(DartLightTeam team)
{
    dart_light_current_team = team;
    return dart_light.RequestSetTeam(team);
}

ResultCode Task_DartLightRequestStatus()
{
    return dart_light.RequestStatus();
}

const DartLightStatus& Task_DartLightGetStatus()
{
    return dart_light.GetStatus();
}

//------------------------------------- STM32 HAL 回调函数 -------------------------------------
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart4) {
        dart_light_uart.OnTxCpltIsr();
    }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart4) {
        dart_light_uart.OnRxCpltIsr();
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart1) {
        host_uart.OnRxErrorIsr(huart->ErrorCode);
    } else if (huart == &huart4) {
        dart_light_uart.OnErrorIsr(huart->ErrorCode);
    }
}
//HAL_UART_Receive_DMA 启动的是循环 DMA，之后这一行把 DMA 的 HT（半传输）和 TC（传输完成）中断都关了。所以RxCpltCallback 不会触发
//这里调用的就是 HAL_UART_ErrorCallback 利用错误事件，用来在出错（溢出）时恢复接收。

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim5) {
        system_timebase.OnTickIsr();
    }
}
