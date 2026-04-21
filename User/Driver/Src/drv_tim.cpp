/**
 * @file drv_tim.cpp
 * @brief 定时器 PWM 驱动实现。
 * @details 提供 PWM 通道启动与比较值更新操作。
 */
#include "drv_tim.h"
#include "Config.h"

namespace DrvTIM {

/**
 * @brief 启动指定 PWM 通道。
 * @param htim 定时器句柄。
 * @param channel PWM 通道。
 * @return 无。
 */
void StartPWM(TIM_HandleTypeDef* htim, uint32_t channel)
{
	if (htim == nullptr) {
		return;
	}

	(void)HAL_TIM_PWM_Start(htim, channel);
}

/**
 * @brief 更新指定通道比较值。
 * @param htim 定时器句柄。
 * @param channel PWM 通道。
 * @param compareValue 比较值（CCR）。
 * @return 无。
 */
void SetCompare(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t compareValue)
{
	if (htim == nullptr) {
		return;
	}

	// 直接使用宏写 CCR，减少控制回路中的函数开销。
	__HAL_TIM_SET_COMPARE(htim, channel, compareValue);
}

} // namespace DrvTIM
