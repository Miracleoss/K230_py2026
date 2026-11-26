#include "Application/app_dart_system.h"

#include "Common/control_command.h"
#include "Common/surface_command.h"
#include "Config/servo_config.h"

// 应用层边界转换：闭环控制器的 ctrl_* 为无量纲 [0, 1]，0.5 为控制中位；
// BodyControlCommand 的 roll/pitch/yaw 为机体轴归一化需求，范围为 [-1, 1]。
// 线性映射为 ctrl=0 -> -1、ctrl=0.5 -> 0、ctrl=1 -> +1；中位值对应安全无偏转。
static float MapControlToBodyAxis(float control)
{
    return 2.0f * control - 1.0f;
}

Class_DartSystem::Class_DartSystem()
    : bmi088_(),
      servos_(),
      imu_heater_(),
      xtail_mixer_(),
      attitude_estimator_(),
      closed_loop_controller_(),
      last_closed_loop_output_(),
      system_status_()
{
}

ResultCode Class_DartSystem::BindBmi088(Class_SpiBus* spi_bus, Class_GpioPin* acc_cs, Class_GpioPin* gyro_cs)
{
    const ResultCode result = bmi088_.Bind(spi_bus, acc_cs, gyro_cs);
    StoreLastError(result);
    return result;
}

ResultCode Class_DartSystem::ProbeBmi088(uint8_t& acc_chip_id, uint8_t& gyro_chip_id)
{
    const ResultCode result = bmi088_.Probe(acc_chip_id, gyro_chip_id);
    StoreLastError(result);
    return result;
}

ResultCode Class_DartSystem::ProbeBmi088WithTrace(
    uint8_t& acc_chip_id,
    uint8_t& gyro_chip_id,
    Bmi088ProbeTrace& trace)
{
    const ResultCode result = bmi088_.ProbeWithTrace(acc_chip_id, gyro_chip_id, trace);
    StoreLastError(result);
    return result;
}

ResultCode Class_DartSystem::InitializeBmi088(Bmi088DelayMsFunction delay_ms)
{
    const ResultCode result = bmi088_.Initialize(delay_ms);
    StoreLastError(result);
    return result;
}

ResultCode Class_DartSystem::ReadBmi088RawSample(Bmi088RawSample& sample)
{
    const ResultCode result = bmi088_.ReadRawSample(sample);
    StoreLastError(result);
    return result;
}

ResultCode Class_DartSystem::BindImuHeater(Class_PwmChannel* heater_pwm)
{
    const ResultCode result = imu_heater_.Bind(heater_pwm);
    StoreLastError(result);
    return result;
}

ResultCode Class_DartSystem::ConfigureImuHeater(const ImuHeaterConfig& config)
{
    const ResultCode result = imu_heater_.Configure(config);
    StoreLastError(result);
    return result;
}

ResultCode Class_DartSystem::StartImuHeater()
{
    const ResultCode result = imu_heater_.Start();
    StoreLastError(result);
    return result;
}

ResultCode Class_DartSystem::UpdateImuHeater(float temperature_deg_c, float dt_s)
{
    const ResultCode result = imu_heater_.Update(temperature_deg_c, dt_s);
    StoreLastError(result);
    return result;
}

ResultCode Class_DartSystem::BindServo(Class_PwmChannel* servo_pwm, const ServoConfig* config)
{
    uint32_t surface_index = 0U;
    if (config == nullptr || !TryGetSurfaceIndex(config->surface_id, surface_index)) {
        StoreLastError(ResultCode::ConfigError);
        return ResultCode::ConfigError;
    }

    const ResultCode result = servos_[surface_index].Bind(servo_pwm, config);
    StoreLastError(result);
    return result;
}

// 应用机体控制指令：将 roll/pitch/yaw 经 X 尾混控转换为各舵面偏转量并下发。
// 任一步失败（指令非法、舵机未绑定、混控失败、下发失败）都会把所有舵机回中，
// 避免部分舵面已动作而其余保持原位导致的失控姿态，属于 fail-safe 保护。
ResultCode Class_DartSystem::ApplyBodyControlCommand(const BodyControlCommand& command)
{
    // 1. 指令校验：valid 标志必须为真，且 roll/pitch/yaw 需归一化在 [-1, 1] 内。
    if (!command.valid ||
        !(command.roll >= -1.0f && command.roll <= 1.0f) ||
        !(command.pitch >= -1.0f && command.pitch <= 1.0f) ||
        !(command.yaw >= -1.0f && command.yaw <= 1.0f)) {
        SetAllServosNeutral();
        StoreLastError(ResultCode::InvalidData);
        return ResultCode::InvalidData;
    }

    // 2. 就绪检查：所有舵机必须已绑定，否则无法下发完整姿态。
    for (uint32_t index = 0U; index < kSurfaceCount; ++index) {
        if (!servos_[index].IsBound()) {
            SetAllServosNeutral();
            StoreLastError(ResultCode::NotReady);
            return ResultCode::NotReady;
        }
    }

    // 3. 混控：把机体坐标系指令转换为各舵面偏转量。
    SurfaceCommand surface_command;
    ResultCode result = xtail_mixer_.Mix(command, surface_command);
    if (result != ResultCode::Ok || !surface_command.valid) {
        SetAllServosNeutral();
        // 混控成功但输出非法时，统一归为 InvalidData 上报。
        if (result == ResultCode::Ok) {
            result = ResultCode::InvalidData;
        }
        StoreLastError(result);
        return result;
    }

    const float deltas[kSurfaceCount] = {
        surface_command.delta1,
        surface_command.delta2,
        surface_command.delta3,
        surface_command.delta4
    };

    // 4. 下发：逐个设置舵面偏转，任一失败立即回中并返回对应错误码。
    for (uint32_t index = 0U; index < kSurfaceCount; ++index) {
        result = servos_[index].SetDelta(deltas[index]);
        if (result != ResultCode::Ok) {
            SetAllServosNeutral();
            StoreLastError(result);
            return result;
        }
    }

    StoreLastError(ResultCode::Ok);
    return ResultCode::Ok;
}

// 应用层闭环入口：过载目标使用机体 +Y/+Z 轴方向的 g，IMU 必须已经是机体坐标系
// 的 SI 数据，dt_s 使用秒。控制器负责输入有效性、样本去重与时序、周期范围和目标钳位；
// 数据年龄超时由 Task 层判定，不属于本入口的职责。
ResultCode Class_DartSystem::UpdateClosedLoop(
    float n_y_c,
    float n_z_c,
    const ImuData& imu,
    float dt_s,
    BodyControlCommand& command,
    bool enable_roll,
    bool enable_pitch_yaw)
{
    // 输出对象默认初始化为 ctrl_* = 0.5、valid = false，失败路径据此保持安全中位。
    ClosedLoopControlOutput output;
    const ResultCode result = closed_loop_controller_.Update(
        n_y_c,
        n_z_c,
        imu,
        dt_s,
        output,
        enable_roll,
        enable_pitch_yaw);

    // 缓存最近一次解算的中间量快照；失败时为安全中性快照（角速度输出为 0）。
    last_closed_loop_output_ = output;

    // 将 ctrl_R/ctrl_P/ctrl_Y 从 [0, 1] 映射为机体 roll/pitch/yaw 的 [-1, 1]；
    // 控制器无效时仍输出 ctrl_* = 0.5，因此三轴保持 0.0 中位而不会产生偏转请求。
    command.roll = MapControlToBodyAxis(output.ctrl_R);
    command.pitch = MapControlToBodyAxis(output.ctrl_P);
    command.yaw = MapControlToBodyAxis(output.ctrl_Y);
    command.timestamp_ms = imu.sample_timestamp_ms;
    command.valid = output.valid;

    // result == Ok 且 output.valid 为真才表示本次样本生成了可用指令；否则 command
    // 保持无效中位，交由上层安全路径回中。时间戳仍记录本次输入样本的时间戳。
    StoreLastError(result);
    return result;
}

void Class_DartSystem::ResetClosedLoop()
{
    // 应用层复位边界：清除控制器内部状态，使后续样本从无历史积分/微分状态开始。
    closed_loop_controller_.Reset();
    last_closed_loop_output_ = ClosedLoopControlOutput();
}

const ClosedLoopControlOutput& Class_DartSystem::GetLastClosedLoopOutput() const
{
    return last_closed_loop_output_;
}

bool Class_DartSystem::IsBmi088Bound() const
{
    return bmi088_.IsBound();
}

bool Class_DartSystem::IsBmi088Initialized() const
{
    return bmi088_.IsInitialized();
}

bool Class_DartSystem::IsImuHeaterBound() const
{
    return imu_heater_.IsBound();
}

bool Class_DartSystem::IsServoBound(uint32_t index) const
{
    if (index >= kSurfaceCount) {
        return false;
    }

    return servos_[index].IsBound();
}

void Class_DartSystem::StoreLastError(ResultCode result)
{
    system_status_.last_error = result;
}

void Class_DartSystem::SetAllServosNeutral()
{
    for (uint32_t index = 0U; index < kSurfaceCount; ++index) {
        if (servos_[index].IsBound()) {
            servos_[index].SetDelta(0.0f);
        }
    }
}
