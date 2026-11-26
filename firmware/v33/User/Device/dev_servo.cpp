#include "Device/dev_servo.h"

Class_Servo::Class_Servo()
    : pwm_(nullptr),
      config_(nullptr)
{
}

ResultCode Class_Servo::Bind(Class_PwmChannel* pwm, const ServoConfig* config)
{
    if (pwm == nullptr || config == nullptr || !pwm->IsBound() ||
        !IsServoPwmChannel(config->pwm_channel_id) ||
        config->physical_min_pulse_us >= config->neutral_pulse_us ||
        config->physical_max_pulse_us <= config->neutral_pulse_us ||
        (config->neutral_pulse_us - config->physical_min_pulse_us) !=
            (config->physical_max_pulse_us - config->neutral_pulse_us)) {
        return ResultCode::ConfigError;
    }

    pwm_ = pwm;
    config_ = config;
    return ResultCode::Ok;
}

/**
 * @brief 检查舵机是否已绑定
 *
 * @return true 已绑定
 *         false 未绑定
 */
bool Class_Servo::IsBound() const
{
    return pwm_ != nullptr && config_ != nullptr;
}

/**
 * @brief 将归一化舵面指令换算为 PWM 脉宽并输出
 *
 * 换算链路：delta 限幅 → 方向反转 → 线性映射到脉宽偏移 → 四舍五入取整 →
 * 物理限幅 → 写入 PWM。映射基准为中立位脉宽，满偏范围为
 * (physical_max_pulse_us - neutral_pulse_us)。
 *
 * @param delta 归一化舵面指令，范围 [-1,+1]，超出时饱和
 * @return ResultCode::Ok          输出成功
 *         ResultCode::ConfigError 未绑定 PWM 或配置
 *         ResultCode::InvalidData delta 为 NaN
 */
ResultCode Class_Servo::SetDelta(float delta)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    // NaN 自比较恒为 false，据此拦截非法输入
    if (delta != delta) {
        return ResultCode::InvalidData;
    }

    // 限幅到 [-1, +1]
    if (delta > 1.0f) {
        delta = 1.0f;
    } else if (delta < -1.0f) {
        delta = -1.0f;
    }

    // 方向反转：reversed 舵机 delta 增大时脉宽应减小
    if (config_->reversed) {
        delta = -delta;
    }

    // 满偏脉宽范围（中立位到最大脉宽的差值），delta 与之相乘得到偏移量
    const uint16_t pulse_range_us =
        config_->physical_max_pulse_us - config_->neutral_pulse_us;
    const float offset = delta * static_cast<float>(pulse_range_us);
    // 四舍五入到最近整数微秒
    const int32_t rounded_offset = static_cast<int32_t>(
        offset >= 0.0f ? offset + 0.5f : offset - 0.5f);
    int32_t pulse_us = static_cast<int32_t>(config_->neutral_pulse_us) + rounded_offset;
    const int32_t minimum_pulse_us = config_->physical_min_pulse_us;
    const int32_t maximum_pulse_us = config_->physical_max_pulse_us;

    // 物理限幅，防止超出舵机允许行程
    if (pulse_us < minimum_pulse_us) {
        pulse_us = minimum_pulse_us;
    } else if (pulse_us > maximum_pulse_us) {
        pulse_us = maximum_pulse_us;
    }

    return pwm_->SetPulseWidthUs(static_cast<uint32_t>(pulse_us));
}
