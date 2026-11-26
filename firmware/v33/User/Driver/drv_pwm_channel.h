#ifndef USER_DRIVER_DRV_PWM_CHANNEL_H
#define USER_DRIVER_DRV_PWM_CHANNEL_H

#include <stdint.h>

#include "Common/common_types.h"
#include "stm32f4xx_hal.h"

/**
 * @brief PWM 通道封装类
 *
 * 对单个定时器 PWM 输出通道进行封装，支持以比较值（tick）、占空比或脉宽
 * 三种方式设置输出，并将 HAL 层错误统一转换为 ResultCode。
 */
class Class_PwmChannel {
public:
    Class_PwmChannel();

    /// 绑定定时器、通道及计数器频率
    ResultCode Bind(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t counter_frequency_hz);

    /// 查询是否已绑定
    bool IsBound() const;

    /// 启动 PWM 输出
    ResultCode Start();

    /// 停止 PWM 输出
    ResultCode Stop();

    /// 直接设置比较寄存器值（tick 数）
    ResultCode SetCompareTicks(uint32_t ticks);

    /// 按占空比设置输出（0.0 ~ 1.0）
    ResultCode SetDutyCycle(float duty_0_to_1);

    /// 按脉宽（us）设置输出
    ResultCode SetPulseWidthUs(uint32_t pulse_width_us);

private:
    /// 将 HAL 状态码转换为 ResultCode
    ResultCode MapHalStatus(HAL_StatusTypeDef status) const;

    TIM_HandleTypeDef* timer_;          ///< 定时器句柄指针
    uint32_t channel_;                  ///< PWM 通道
    uint32_t counter_frequency_hz_;     ///< 计数器频率（Hz）
    uint32_t max_compare_ticks_;        ///< 最大比较值（ARR）
};

#endif /* USER_DRIVER_DRV_PWM_CHANNEL_H */
