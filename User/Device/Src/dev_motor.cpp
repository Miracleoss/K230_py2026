/**
 * @file dev_motor.cpp
 * @brief 电机设备层实现。
 * @details 提供油门到 PWM 占空比映射，以及急停控制。
 */
#include "dev_motor.h"

#include "Config.h"

#include "tim.h"

#include "drv_tim.h"

/**
 * @brief 构造函数实现。
 * @param htim PWM 定时器句柄。
 * @param channel PWM 通道。
 */
Motor::Motor(TIM_HandleTypeDef* htim, uint32_t channel)
	: htim_(htim),
	  channel_(channel),
	  throttle_(0.0f)
{
	if (htim_ == nullptr) {
		// 默认绑定 htim3，便于快速接入单路电机输出。
		htim_ = &htim3;
	}
}

/**
 * @brief 初始化电机 PWM。
 * @return true：初始化成功；false：初始化失败。
 */
bool Motor::Init()
{
	if (htim_ == nullptr) {
		return false;
	}

	// 启动 PWM 并清零输出，防止上电突发转速。
	DrvTIM::StartPWM(htim_, channel_);
	EmergencyStop();
	return true;
}

/**
 * @brief 设置油门。
 * @param duty 油门值 [0,1]。
 * @return 无。
 */
void Motor::SetThrottle(float duty)
{
	if (htim_ == nullptr) {
		return;
	}

	// 限幅后映射为比较值并写入 PWM。
	throttle_ = ClampDuty(duty);
	DrvTIM::SetCompare(htim_, channel_, DutyToCompare(htim_, throttle_));
}

/**
 * @brief 急停。
 * @return 无。
 */
void Motor::EmergencyStop()
{
	throttle_ = 0.0f;
	if (htim_ != nullptr) {
		// 直接将比较值清零，输出占空比归零。
		DrvTIM::SetCompare(htim_, channel_, 0U);
	}
}

/**
 * @brief 绑定新的输出端口。
 * @param htim 定时器句柄。
 * @param channel 通道。
 * @return 无。
 */
void Motor::BindOutput(TIM_HandleTypeDef* htim, uint32_t channel)
{
	htim_ = htim;
	channel_ = channel;
}

/**
 * @brief 获取当前油门值。
 * @return float 当前油门。
 */
float Motor::GetThrottle() const
{
	return throttle_;
}

/**
 * @brief 油门限幅。
 * @param duty 输入油门。
 * @return float 限幅后的油门。
 */
float Motor::ClampDuty(float duty)
{
	if (duty < 0.0f) {
		return 0.0f;
	}

	if (duty > 1.0f) {
		return 1.0f;
	}

	return duty;
}

/**
 * @brief 占空比映射到比较值。
 * @param htim 定时器句柄。
 * @param duty 占空比 [0,1]。
 * @return uint32_t 比较值。
 */
uint32_t Motor::DutyToCompare(TIM_HandleTypeDef* htim, float duty)
{
	if (htim == nullptr) {
		return 0U;
	}

	// Compare = duty * ARR，采用四舍五入。
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
	float compare_f = duty * static_cast<float>(arr);
	uint32_t compare = static_cast<uint32_t>(compare_f + 0.5f);

	if (compare > arr) {
		compare = arr;
	}

	return compare;
}
