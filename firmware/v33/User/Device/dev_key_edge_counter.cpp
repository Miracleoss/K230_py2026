#include "Device/dev_key_edge_counter.h"

Class_KeyEdgeCounter::Class_KeyEdgeCounter()
    : pin_(nullptr),
      edge_threshold_(5U),
      debounce_ms_(20U),
      window_ms_(3000U),
      current_raw_state_(true),
      stable_state_(true),
      last_state_change_ms_(0U),
      edge_count_(0U),
      first_edge_ms_(0U)
{
}

ResultCode Class_KeyEdgeCounter::Bind(
    Class_GpioPin* pin,
    uint8_t edge_threshold,
    uint32_t debounce_ms,
    uint32_t window_ms)
{
    if (pin == nullptr || !pin->IsBound() || edge_threshold == 0U) {
        return ResultCode::ConfigError;
    }

    pin_ = pin;
    edge_threshold_ = edge_threshold;
    debounce_ms_ = debounce_ms;
    window_ms_ = window_ms;

    // 默认上拉为高电平
    bool initial_level = true;
    pin_->Read(initial_level);
    current_raw_state_ = initial_level;
    stable_state_ = initial_level;
    last_state_change_ms_ = 0U;
    edge_count_ = 0U;
    first_edge_ms_ = 0U;

    return ResultCode::Ok;
}

bool Class_KeyEdgeCounter::IsBound() const
{
    return pin_ != nullptr;
}

bool Class_KeyEdgeCounter::Update(uint32_t now_ms)
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
    } else {
        if (now_ms - last_state_change_ms_ >= debounce_ms_) {
            if (current_raw_state_ != stable_state_) {
                const bool previous_state = stable_state_;
                stable_state_ = current_raw_state_;

                // 3. 判断是否为有效下降沿（从高变低，即按下动作）
                if (previous_state && !stable_state_) {
                    if (edge_count_ == 0U || (now_ms - first_edge_ms_ > window_ms_)) {
                        edge_count_ = 1U;
                        first_edge_ms_ = now_ms;
                    } else {
                        ++edge_count_;
                    }

                    if (edge_count_ >= edge_threshold_) {
                        edge_count_ = 0U;
                        first_edge_ms_ = 0U;
                        triggered = true;
                    }
                }
            }
        }
    }

    // 4. 检查累计时间窗口是否超时
    if (edge_count_ > 0U && (now_ms - first_edge_ms_ > window_ms_)) {
        edge_count_ = 0U;
        first_edge_ms_ = 0U;
    }

    return triggered;
}

uint8_t Class_KeyEdgeCounter::GetEdgeCount() const
{
    return edge_count_;
}

void Class_KeyEdgeCounter::Reset()
{
    edge_count_ = 0U;
    first_edge_ms_ = 0U;
}

