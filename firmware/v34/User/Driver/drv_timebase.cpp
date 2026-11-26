#include "Driver/drv_timebase.h"

Class_Timebase::Class_Timebase()
    : timer_(nullptr),
      uptime_ms_(0U)
{
}

ResultCode Class_Timebase::Bind(TIM_HandleTypeDef* timer)
{
    if (timer == nullptr) {
        return ResultCode::ConfigError;
    }

    timer_ = timer;
    uptime_ms_ = 0U;
    return ResultCode::Ok;
}

bool Class_Timebase::IsBound() const
{
    return timer_ != nullptr;
}

ResultCode Class_Timebase::Start()
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    uptime_ms_ = 0U;
    if (timer_->Instance != nullptr) {
        __HAL_TIM_SET_COUNTER(timer_, 0U);
        __HAL_TIM_CLEAR_FLAG(timer_, TIM_FLAG_UPDATE);
    }

    return MapHalStatus(HAL_TIM_Base_Start_IT(timer_));
}

ResultCode Class_Timebase::Stop()
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    return MapHalStatus(HAL_TIM_Base_Stop_IT(timer_));
}

void Class_Timebase::OnTickIsr()
{
    ++uptime_ms_;
}

uint32_t Class_Timebase::NowMs() const
{
    return uptime_ms_;
}

uint64_t Class_Timebase::NowUs() const
{
    if (!IsBound() || timer_->Instance == nullptr ||
        (timer_->Instance->CR1 & TIM_CR1_CEN) == 0U) {
        return static_cast<uint64_t>(uptime_ms_) * 1000ULL;
    }

    /*
     * 无锁读取毫秒计数与 1 MHz（0..999 us）硬件计数器：
     * 1. 若读取期间发生中断抢占更新了 uptime_ms_，ms_first != ms_second 触发重试；
     * 2. 若硬件计数刚跨过 999->0 置位 UIF 但中断尚未执行，update_pending 为真，
     *    此时采用翻转后的 cnt_second 并对毫秒计数补偿 +1，确保严格单调不回退。
     */
    uint32_t ms_first = 0U;
    uint32_t cnt_first = 0U;
    bool update_pending = false;
    uint32_t cnt_second = 0U;
    uint32_t ms_second = 0U;
    do {
        ms_first = uptime_ms_;
        cnt_first = __HAL_TIM_GET_COUNTER(timer_);
        update_pending = (__HAL_TIM_GET_FLAG(timer_, TIM_FLAG_UPDATE) != RESET);
        cnt_second = __HAL_TIM_GET_COUNTER(timer_);
        ms_second = uptime_ms_;
    } while (ms_first != ms_second);

    if (update_pending) {
        return (static_cast<uint64_t>(ms_first) + 1ULL) * 1000ULL +
               static_cast<uint64_t>(cnt_second);
    }

    return static_cast<uint64_t>(ms_first) * 1000ULL +
           static_cast<uint64_t>(cnt_first);
}

ResultCode Class_Timebase::MapHalStatus(HAL_StatusTypeDef status) const
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
