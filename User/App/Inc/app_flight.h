#pragma once

#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif

/**
 * @brief 飞控应用层初始化。
 * @return true 初始化成功；false 初始化失败。
 */
bool App_Flight_Init(void);

/**
 * @brief 飞控任务函数（建议 100Hz 周期调用）。
 * @return 无。
 */
void App_Flight_Task(void);

#ifdef __cplusplus
}
#endif

