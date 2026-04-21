/**
 * @file drv_tim.h
 * @brief 定时器 PWM 驱动层接口。
 * @details 提供 PWM 启动与比较值更新接口，供电机/舵机设备层调用。
 */
#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}

namespace DrvTIM {

/**
 * @brief 启动指定通道 PWM 输出。
 * @param htim 定时器句柄。
 * @param channel PWM 通道。
 * @return 无。
 */
void StartPWM(TIM_HandleTypeDef* htim, uint32_t channel);

/**
 * @brief 更新比较寄存器（CCR）以调整占空比。
 * @param htim 定时器句柄。
 * @param channel PWM 通道。
 * @param compareValue 比较值（CCR）。
 * @return 无。
 */
void SetCompare(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t compareValue);

} // namespace DrvTIM
#endif
