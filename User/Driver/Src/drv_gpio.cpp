/**
 * @file drv_gpio.cpp
 * @brief GPIO 驱动实现。
 * @details 对 HAL GPIO 基础操作进行薄封装，供设备层统一调用。
 */
#include "drv_gpio.h"
#include "Config.h"

namespace DrvGPIO {

/**
 * @brief 将指定引脚置高。
 * @param GPIOx GPIO 端口。
 * @param GPIO_Pin GPIO 引脚掩码。
 * @return 无。
 */
void SetPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	// 保持驱动接口稳定，避免上层直接耦合 HAL 命名。
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

/**
 * @brief 将指定引脚置低。
 * @param GPIOx GPIO 端口。
 * @param GPIO_Pin GPIO 引脚掩码。
 * @return 无。
 */
void ResetPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 翻转指定引脚电平。
 * @param GPIOx GPIO 端口。
 * @param GPIO_Pin GPIO 引脚掩码。
 * @return 无。
 */
void TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}

/**
 * @brief 读取指定引脚电平。
 * @param GPIOx GPIO 端口。
 * @param GPIO_Pin GPIO 引脚掩码。
 * @return GPIO_PinState 引脚状态。
 */
GPIO_PinState ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

} // namespace DrvGPIO
