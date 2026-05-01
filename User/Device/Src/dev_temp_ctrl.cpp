/**
 * @file dev_temp_ctrl.cpp
 * @brief BMI088 温度控制器实现。
 * @details PID 闭环控制加热 PWM 占空比，维持 BMI088 在目标温度。
 */
#include "dev_temp_ctrl.h"
#include "drv_tim.h"

TempCtrl::TempCtrl(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t arr)
	: htim_(htim), channel_(channel), arr_(arr)
{
}

bool TempCtrl::Init()
{
	if (htim_ == nullptr) {
		return false;
	}

	DrvTIM::StartPWM(htim_, channel_);
	DrvTIM::SetCompare(htim_, channel_, 0U);
	return true;
}

void TempCtrl::SetTarget(float targetC)
{
	target_temp_ = targetC;
}

uint32_t TempCtrl::Update(float currentC)
{
	current_temp_ = currentC;

	float error = target_temp_ - currentC;

	// 死区：误差足够小时保持当前输出，避免频繁抖动。
	if (error > -kDeadZone && error < kDeadZone) {
		error = 0.0f;
	}

	// 积分累加 + 限幅（抗积分饱和）。
	integral_ += error;
	if (integral_ > kIntegralLimit) {
		integral_ = kIntegralLimit;
	} else if (integral_ < -kIntegralLimit) {
		integral_ = -kIntegralLimit;
	}

	// 微分项（首次更新跳过）。
	float derivative = 0.0f;
	if (!first_update_) {
		derivative = error - prev_error_;
	} else {
		first_update_ = false;
	}
	prev_error_ = error;

	// PID 输出。
	float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

	// 输出限幅到 [0, arr]。
	if (output < 0.0f) {
		output = 0.0f;
		integral_ -= error; // 反向饱和时回退积分
	}
	float max_out = static_cast<float>(arr_);
	if (output > max_out) {
		output = max_out;
		integral_ -= error; // 正向饱和时回退积分
	}

	pwm_duty_ = static_cast<uint32_t>(output);
	DrvTIM::SetCompare(htim_, channel_, pwm_duty_);

	return pwm_duty_;
}
