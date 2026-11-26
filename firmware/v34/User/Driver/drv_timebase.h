#ifndef USER_DRIVER_DRV_TIMEBASE_H
#define USER_DRIVER_DRV_TIMEBASE_H

#include <stdint.h>

#include "Common/common_types.h"
#include "stm32f4xx_hal.h"

/**
 * @brief 时基封装类
 *
 * 基于 1 MHz（1 us/tick）硬件定时器计数与 1 kHz（1000 ticks）溢出中断，
 * 同时提供毫秒级（NowMs）与硬件 1 us 分辨率微秒级（NowUs）单调时间基准。
 */
class Class_Timebase {
public:
    Class_Timebase();

    /// 绑定定时器
    ResultCode Bind(TIM_HandleTypeDef* timer);

    /// 查询是否已绑定
    bool IsBound() const;

    /// 清零计数并启动时基（定时器中断）
    ResultCode Start();

    /// 停止时基
    ResultCode Stop();

    /// 定时器溢出中断服务函数（每 1000 us 即 1 ms 调用一次，累加毫秒计数）
    void OnTickIsr();

    /// 获取当前运行时间（ms）
    uint32_t NowMs() const;

    /// 获取当前运行时间（us，硬件 1 us 计数分辨率，无锁防跨拍翻转）
    uint64_t NowUs() const;

private:
    /// 将 HAL 状态码转换为 ResultCode
    ResultCode MapHalStatus(HAL_StatusTypeDef status) const;

    TIM_HandleTypeDef* timer_;         ///< 定时器句柄指针
    volatile uint32_t uptime_ms_;      ///< 运行时间计数（ms），volatile 修饰以在中断中安全访问
};

#endif /* USER_DRIVER_DRV_TIMEBASE_H */
