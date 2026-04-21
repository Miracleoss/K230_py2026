/**
 * @file dev_servo.cpp
 * @brief 舵机设备层实现。
 * @details 完成 4 路舵偏角限幅、脉宽映射和 TIM1 PWM 输出。
 */
#include "dev_servo.h"

#include "Config.h"

#include "tim.h"

#include "drv_tim.h"

namespace {

// 舵机角度映射区间下限（度）。
constexpr float kServoAngleMinDeg = FcParam::Servo::kMapAngleMinDeg;
// 舵机角度映射区间上限（度）。
constexpr float kServoAngleMaxDeg = FcParam::Servo::kMapAngleMaxDeg;
// 飞控输出硬限幅（度），用于机械保护。
constexpr float kServoLimitDeg = FcParam::Servo::kAngleLimitDeg;

// 舵机脉宽最小值（us），对应 -45°。
constexpr float kPulseMinUs = FcParam::Servo::kPulseMinUs;
// 舵机脉宽最大值（us），对应 +45°。
constexpr float kPulseMaxUs = FcParam::Servo::kPulseMaxUs;

// 舵面数量。
constexpr uint8_t kFinCount = 4U;
// 4 路舵机对应 TIM1 通道表。
constexpr uint32_t kServoChannels[kFinCount] = {
	PinDef::Servo::SERVO_CH1,
	PinDef::Servo::SERVO_CH2,
	PinDef::Servo::SERVO_CH3,
	PinDef::Servo::SERVO_CH4,
};

} // namespace

/**
 * @brief 构造函数实现。
 * @param htim PWM 定时器句柄。
 */
Servo::Servo(TIM_HandleTypeDef* htim)
	: htim_(htim)
{
	if (htim_ == nullptr) {
		// 默认使用 TIM1 输出 4 路舵机 PWM。
		htim_ = &htim1;
	}
}

/**
 * @brief 初始化舵机 PWM 输出。
 * @return true：初始化成功；false：初始化失败。
 */
bool Servo::Init()
{
	if (htim_ == nullptr) {
		return false;
	}

	// 启动 CH1~CH4 PWM 通道。
	for (uint8_t i = 0U; i < kFinCount; ++i) {
		DrvTIM::StartPWM(htim_, kServoChannels[i]);
	}

	// 上电输出中位角，避免突变。
	float neutral[kFinCount] = {0.0f, 0.0f, 0.0f, 0.0f};
	SetFinAngles(neutral);
	return true;
}

/**
 * @brief 设置 4 路舵偏角。
 * @param delta 舵偏角数组（单位：度）。
 * @return 无。
 */
void Servo::SetFinAngles(float delta[4])
{
	if ((htim_ == nullptr) || (delta == nullptr)) {
		return;
	}

	for (uint8_t i = 0U; i < kFinCount; ++i) {
		// 角度限幅 -> 脉宽映射 -> 比较值映射。
		float limited_angle = ClampAngleDeg(delta[i]);
		float pulse_us = AngleDegToPulseUs(limited_angle);
		uint32_t compare = PulseUsToCompare(htim_, pulse_us);
		// 写入对应通道的 PWM 比较值。
		DrvTIM::SetCompare(htim_, kServoChannels[i], compare);
	}
}

/**
 * @brief 绑定定时器句柄。
 * @param htim 目标定时器句柄。
 * @return 无。
 */
void Servo::BindTimer(TIM_HandleTypeDef* htim)
{
	htim_ = htim;
}

/**
 * @brief 舵偏角硬限幅。
 * @param angle_deg 输入角度。
 * @return float 限幅后角度。
 */
float Servo::ClampAngleDeg(float angle_deg)
{
	if (angle_deg > kServoLimitDeg) {
		return kServoLimitDeg;
	}

	if (angle_deg < -kServoLimitDeg) {
		return -kServoLimitDeg;
	}

	return angle_deg;
}

/**
 * @brief 舵偏角映射为脉宽。
 * @param angle_deg 舵偏角（度）。
 * @return float 脉宽（us）。
 */
float Servo::AngleDegToPulseUs(float angle_deg)
{
	// 线性映射：[-45,45] -> [500us,2500us]。
	float normalized = (angle_deg - kServoAngleMinDeg) / (kServoAngleMaxDeg - kServoAngleMinDeg);
	return kPulseMinUs + normalized * (kPulseMaxUs - kPulseMinUs);
}

/**
 * @brief 脉宽映射为定时器比较值。
 * @param htim 定时器句柄。
 * @param pulse_us 脉宽（us）。
 * @return uint32_t 比较值。
 */
uint32_t Servo::PulseUsToCompare(TIM_HandleTypeDef* htim, float pulse_us)
{
	if (htim == nullptr) {
		return 0U;
	}

	// 读取定时器输入时钟、预分频与自动重装载参数。
	uint32_t timer_clk_hz = GetTimerClockHz(htim->Instance);
	uint32_t psc = htim->Init.Prescaler;
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

	if (timer_clk_hz == 0U) {
		return 0U;
	}

	uint32_t tick_hz = timer_clk_hz / (psc + 1U);
	if (tick_hz == 0U) {
		return 0U;
	}

	// Compare = pulse_us * tick_hz / 1e6（四舍五入）。
	float compare_f = (pulse_us * static_cast<float>(tick_hz)) / 1000000.0f;
	uint32_t compare = static_cast<uint32_t>(compare_f + 0.5f);

	if (compare > arr) {
		compare = arr;
	}

	return compare;
}

/**
 * @brief 获取指定定时器输入时钟频率。
 * @param instance 定时器实例。
 * @return uint32_t 时钟频率（Hz）。
 */
uint32_t Servo::GetTimerClockHz(TIM_TypeDef* instance)
{
	if (instance == nullptr) {
		return 0U;
	}

	if (instance == TIM1) {
		// 高级定时器位于 APB2，总线分频不为 1 时计时频率翻倍。
		uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
		uint32_t ppre2 = RCC->CFGR & RCC_CFGR_PPRE2;
		return (ppre2 == RCC_CFGR_PPRE2_DIV1) ? pclk2 : (pclk2 * 2U);
	}

	// 通用定时器位于 APB1，总线分频不为 1 时计时频率翻倍。
	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	uint32_t ppre1 = RCC->CFGR & RCC_CFGR_PPRE1;
	return (ppre1 == RCC_CFGR_PPRE1_DIV1) ? pclk1 : (pclk1 * 2U);
}
