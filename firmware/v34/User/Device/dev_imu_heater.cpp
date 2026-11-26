#include "Device/dev_imu_heater.h"

#include <float.h>

/* 基础数值 helper：在不依赖运行库分类函数的前提下，拒绝超出 float 有限范围的输入。 */
static bool IsFiniteFloat(float value)
{
    return value <= FLT_MAX && value >= -FLT_MAX;
}

/* 通用限幅 helper：将积分状态和输出限制在已校验的闭区间内。 */
static float ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

ImuHeaterConfig MakeDefaultImuHeaterConfig()
{
    ImuHeaterConfig config;
    config.target_temperature_deg_c = 45.0f;
    config.max_temperature_deg_c = 55.0f;
    config.kp = 5.0f;
    config.ki = 0.0f;
    config.integral_min = 0.0f;
    config.integral_max = 100.0f;
    config.duty_min = 0.0f;
    config.duty_max = 1.0f;
    config.dt_min_s = 0.001f;
    config.dt_max_s = 0.100f;
    return config;
}

Class_ImuHeater::Class_ImuHeater()
    : pwm_(nullptr),
      config_(MakeDefaultImuHeaterConfig()),
      integral_(0.0f)
{
}

ResultCode Class_ImuHeater::Bind(Class_PwmChannel* pwm)
{
    if (pwm == nullptr) {
        return ResultCode::ConfigError;
    }

    pwm_ = pwm;
    return ResultCode::Ok;
}

bool Class_ImuHeater::IsBound() const
{
    return pwm_ != nullptr;
}

ResultCode Class_ImuHeater::Configure(const ImuHeaterConfig& config)
{
    config_ = config;
    integral_ = 0.0f;
    return ResultCode::Ok;
}

ResultCode Class_ImuHeater::Start()
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    // 先输出 0 占空比再启动，避免启动瞬间残留上一轮比较值导致误加热。
    ResultCode result = pwm_->SetDutyCycle(0.0f);
    if (result != ResultCode::Ok) {
        return result;
    }
    return pwm_->Start();
}

ResultCode Class_ImuHeater::Stop()
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    return pwm_->Stop();
}

ResultCode Class_ImuHeater::Update(float temperature_deg_c, float dt_s)
{
    if (!IsBound()) {
        return ResultCode::NotReady;
    }
    if (!IsFiniteFloat(temperature_deg_c) || !IsFiniteFloat(dt_s)) {
        return ResultCode::InvalidData;
    }

    const float clamped_dt = ClampFloat(dt_s, config_.dt_min_s, config_.dt_max_s);

    // 超温保护：达到或超过安全上限时关断加热并清空积分，避免积分继续累积。
    if (temperature_deg_c >= config_.max_temperature_deg_c) {
        integral_ = 0.0f;
        return pwm_->SetDutyCycle(0.0f);
    }

    const float error = config_.target_temperature_deg_c - temperature_deg_c;
    integral_ = ClampFloat(
        integral_ + error * clamped_dt,
        config_.integral_min,
        config_.integral_max);

    const float duty = ClampFloat(
        config_.kp * error + config_.ki * integral_,
        config_.duty_min,
        config_.duty_max);

    return pwm_->SetDutyCycle(duty);
}
