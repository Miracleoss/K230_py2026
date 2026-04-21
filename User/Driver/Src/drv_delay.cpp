/**
 * @file drv_delay.cpp
 * @brief 延时驱动实现。
 * @details 通过 HAL_Delay 与 SysTick 计数器实现毫秒/微秒阻塞延时。
 */
#include "drv_delay.h"
#include "Config.h"

namespace {

/**
 * @brief 获取 SysTick 实际计数时钟频率。
 * @return SysTick 时钟频率，单位 Hz。
 */
uint32_t GetSysTickClockHz()
{
	// 获取系统 HCLK，用于换算 SysTick 计数频率。
	uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();
	if ((SysTick->CTRL & SysTick_CTRL_CLKSOURCE_Msk) != 0U) {
		// CLKSOURCE=1：SysTick 时钟源为 HCLK。
		return hclk_hz;
	}

	// CLKSOURCE=0：SysTick 时钟源为 HCLK/8。
	return (hclk_hz / 8U);
}

} // namespace

/**
 * @brief 毫秒阻塞延时实现。
 * @param ms 延时时间，单位 ms。
 * @return 无。
 */
void Delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

/**
 * @brief 微秒阻塞延时实现。
 * @param us 延时时间，单位 us。
 * @return 无。
 */
void Delay_us(uint32_t us)
{
	if (us == 0U) {
		return;
	}

	if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0U) {
		// SysTick 未使能时，退化为毫秒延时。
		HAL_Delay((us + 999U) / 1000U);
		return;
	}

	// 计算每 1us 对应的 SysTick tick 数。
	uint32_t systick_hz = GetSysTickClockHz();
	uint32_t ticks_per_us = systick_hz / 1000000U;

	if (ticks_per_us == 0U) {
		// 频率异常过低时，退化为毫秒延时。
		HAL_Delay((us + 999U) / 1000U);
		return;
	}

	// 目标总 tick 数与累计 tick 数。
	uint64_t target_ticks = static_cast<uint64_t>(us) * static_cast<uint64_t>(ticks_per_us);
	uint64_t elapsed_ticks = 0U;
	// SysTick 从 LOAD 递减到 0，再重装载。
	uint32_t reload = SysTick->LOAD + 1U;
	uint32_t previous = SysTick->VAL;

	while (elapsed_ticks < target_ticks) {
		uint32_t current = SysTick->VAL;

		if (current == previous) {
			continue;
		}

		if (previous >= current) {
			// 正常递减区间。
			elapsed_ticks += static_cast<uint64_t>(previous - current);
		} else {
			// 处理计数器从 0 回绕到 LOAD 的情况。
			elapsed_ticks += static_cast<uint64_t>(previous + (reload - current));
		}

		previous = current;
	}
}
