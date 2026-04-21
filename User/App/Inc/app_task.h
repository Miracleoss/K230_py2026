#pragma once

#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif

#include <stdint.h>
#include "main.h"

/**
 * @brief 全局毫秒计数（由 TIM2 中断维护，步进为 10ms）。
 */
extern volatile uint32_t t_ms;

/**
 * @brief 10ms 任务触发标志。
 */
extern volatile bool task_10ms_flag;

/**
 * @brief 20ms 任务触发标志。
 */
extern volatile bool task_20ms_flag;

/**
 * @brief 任务拥塞计数（用于性能监控）。
 */
extern volatile uint32_t task_overload_count;

/**
 * @brief HAL 定时器周期中断回调。
 * @details 在 TIM2=100Hz 中断下维护 t_ms，并置位任务触发标志。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

/**
 * @brief 任务系统初始化。
 * @details 初始化飞控应用并启动 TIM2 周期中断调度。
 * @return true 初始化成功；false 初始化失败。
 */
bool App_Task_Init(void);

/**
 * @brief 前后台任务循环。
 * @details 建议在 main 的 while(1) 中反复调用。
 */
void Task_Loop(void);

#ifdef __cplusplus
}
#endif

