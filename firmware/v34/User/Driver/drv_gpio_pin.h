#ifndef USER_DRIVER_DRV_GPIO_PIN_H
#define USER_DRIVER_DRV_GPIO_PIN_H

#include <stdint.h>

#include "Common/common_types.h"
#include "stm32f4xx_hal.h"

/**
 * @brief GPIO 引脚封装类
 *
 * 对单个 GPIO 引脚进行面向对象封装，提供绑定、置高、置低、写入与读取操作，
 * 并将 HAL 层的错误统一转换为 ResultCode。
 */
class Class_GpioPin {
public:
    Class_GpioPin();

    /// 绑定一个 GPIO 端口与引脚
    ResultCode Bind(GPIO_TypeDef* port, uint16_t pin);

    /// 查询是否已绑定
    bool IsBound() const;

    /// 输出高电平
    ResultCode SetHigh();

    /// 输出低电平
    ResultCode SetLow();

    /// 输出指定电平（true 为高，false 为低）
    ResultCode Write(bool high);

    /// 读取引脚当前电平
    ResultCode Read(bool& high) const;

private:
    GPIO_TypeDef* port_;  ///< GPIO 端口指针
    uint16_t pin_;        ///< 引脚编号
};

#endif /* USER_DRIVER_DRV_GPIO_PIN_H */
