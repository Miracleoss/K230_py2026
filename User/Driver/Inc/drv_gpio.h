/**
 * @file drv_gpio.h
 * @brief GPIO 驱动层接口。
 * @details 提供最小化 GPIO 读写封装，隔离上层与 HAL 直接耦合。
 */
#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}
#endif

namespace DrvGPIO {

/**
 * @brief 将 GPIO 引脚置高。
 * @param GPIOx GPIO 端口基地址。
 * @param GPIO_Pin GPIO 引脚掩码。
 * @return 无。
 */
void SetPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/**
 * @brief 将 GPIO 引脚置低。
 * @param GPIOx GPIO 端口基地址。
 * @param GPIO_Pin GPIO 引脚掩码。
 * @return 无。
 */
void ResetPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/**
 * @brief 翻转 GPIO 引脚电平。
 * @param GPIOx GPIO 端口基地址。
 * @param GPIO_Pin GPIO 引脚掩码。
 * @return 无。
 */
void TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/**
 * @brief 读取 GPIO 引脚当前电平。
 * @param GPIOx GPIO 端口基地址。
 * @param GPIO_Pin GPIO 引脚掩码。
 * @return GPIO_PinState 引脚状态（高/低电平）。
 */
GPIO_PinState ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

} // namespace DrvGPIO
