#include "Driver/drv_pwm_channel.h"

Class_PwmChannel::Class_PwmChannel()
    : timer_(nullptr),
      channel_(0U),
      counter_frequency_hz_(0U),
      max_compare_ticks_(0U)
{
}

ResultCode Class_PwmChannel::Bind(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t counter_frequency_hz)
{
    if (timer == nullptr || counter_frequency_hz == 0U) {
        return ResultCode::ConfigError;
    }

    timer_ = timer;
    channel_ = channel;
    counter_frequency_hz_ = counter_frequency_hz;
    max_compare_ticks_ = timer->Init.Period;
    return ResultCode::Ok;
}

bool Class_PwmChannel::IsBound() const
{
    return timer_ != nullptr && counter_frequency_hz_ != 0U;
}

ResultCode Class_PwmChannel::Start()
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    return MapHalStatus(HAL_TIM_PWM_Start(timer_, channel_));
}

ResultCode Class_PwmChannel::Stop()
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    return MapHalStatus(HAL_TIM_PWM_Stop(timer_, channel_));
}

ResultCode Class_PwmChannel::SetCompareTicks(uint32_t ticks)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    if (ticks > max_compare_ticks_) {
        return ResultCode::InvalidData;
    }

    __HAL_TIM_SET_COMPARE(timer_, channel_, ticks);
    return ResultCode::Ok;
}

ResultCode Class_PwmChannel::SetDutyCycle(float duty_0_to_1)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    if (!(duty_0_to_1 >= 0.0f && duty_0_to_1 <= 1.0f)) {
        return ResultCode::InvalidData;
    }

    /*
     * 将比较值限制在显式的 CCR 范围 [0, ARR] 内。占空比 1.0 对应 ARR，
     * 避免出现 ARR+1 的比较值（该值在不同定时器位宽与 PWM 模式下不可移植）。
     */
    const float scaled_ticks = duty_0_to_1 * static_cast<float>(max_compare_ticks_);
    const uint32_t compare_ticks = static_cast<uint32_t>(scaled_ticks + 0.5f);
    return SetCompareTicks(compare_ticks);
}

ResultCode Class_PwmChannel::SetPulseWidthUs(uint32_t pulse_width_us)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    const uint64_t ticks =
        (static_cast<uint64_t>(pulse_width_us) * static_cast<uint64_t>(counter_frequency_hz_)) /
        1000000ULL;

    if (ticks > static_cast<uint64_t>(max_compare_ticks_)) {
        return ResultCode::InvalidData;
    }

    return SetCompareTicks(static_cast<uint32_t>(ticks));
}

ResultCode Class_PwmChannel::MapHalStatus(HAL_StatusTypeDef status) const
{
    switch (status) {
    case HAL_OK:
        return ResultCode::Ok;
    case HAL_TIMEOUT:
        return ResultCode::Timeout;
    case HAL_BUSY:
        return ResultCode::NotReady;
    case HAL_ERROR:
    default:
        return ResultCode::HardwareError;
    }
}
