/**
 * @file drv_delay.h
 * @brief 延时驱动层接口。
 * @details 封装毫秒级与微秒级阻塞延时，供上层模块统一调用。
 */
#pragma once

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}
#endif

/**
 * @brief 毫秒级阻塞延时。
 * @param ms 延时时间，单位 ms。
 * @return 无。
 */
void Delay_ms(uint32_t ms);

/**
 * @brief 微秒级阻塞延时。
 * @details 使用 SysTick 的 VAL/LOAD 计数实现，不依赖额外硬件定时器。
 * @param us 延时时间，单位 us。
 * @return 无。
 */
void Delay_us(uint32_t us);
