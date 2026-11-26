#include "Device/dev_key_long_press.h"

Class_KeyLongPress::Class_KeyLongPress()
    : pin_(nullptr),
      debounce_ms_(20U),
      hold_ms_(1500U),
      current_raw_state_(true),
      stable_state_(true),
      last_state_change_ms_(0U),
      low_start_ms_(0U),
      reported_(false)
{
}

ResultCode Class_KeyLongPress::Bind(
    Class_GpioPin* pin,
    uint32_t debounce_ms,
    uint32_t hold_ms)
{
    if (pin == nullptr || !pin->IsBound() || hold_ms == 0U) {
        return ResultCode::ConfigError;
    }

    pin_ = pin;
    debounce_ms_ = debounce_ms;
    hold_ms_ = hold_ms;

    // 默认上拉为高电平
    bool initial_level = true;
    pin_->Read(initial_level);
    current_raw_state_ = initial_level;
    stable_state_ = initial_level;
    last_state_change_ms_ = 0U;
    low_start_ms_ = 0U;
    reported_ = false;

    return ResultCode::Ok;
}

bool Class_KeyLongPress::IsBound() const
{
    return pin_ != nullptr;
}

bool Class_KeyLongPress::IsHeldLow() const
{
    return !stable_state_;
}

bool Class_KeyLongPress::Update(uint32_t now_ms)
{
    if (!IsBound()) {
        return false;
    }

    // 1. 读取引脚瞬时电平（高电平表示松开，低电平表示按下）
    bool pin_high = true;
    if (pin_->Read(pin_high) != ResultCode::Ok) {
        return false;
    }

    bool triggered = false;

    // 2. 软件消抖滤波
    if (pin_high != current_raw_state_) {
        current_raw_state_ = pin_high;
        last_state_change_ms_ = now_ms;
    } else if (now_ms - last_state_change_ms_ >= debounce_ms_) {
        if (current_raw_state_ != stable_state_) {
            stable_state_ = current_raw_state_;
            if (!stable_state_) {
                // 高→低：按下，开始计时
                low_start_ms_ = now_ms;
                reported_ = false;
            } else {
                // 低→高：松开，结束计时
                low_start_ms_ = 0U;
                reported_ = false;
            }
        }
    }

    // 3. 低电平保持达到阈值时单次触发
    if (!stable_state_ && low_start_ms_ != 0U && !reported_ &&
        (now_ms - low_start_ms_ >= hold_ms_)) {
        reported_ = true;
        triggered = true;
    }

    return triggered;
}
