/**
 * @file dev_temp_ctrl.h
 * @brief BMI088 温度控制器接口。
 * @details 基于 PID 算法，通过 TIM3 CH3 PWM 输出控制加热电路，
 *          使 BMI088 工作在目标温度以减小温漂。
 */
#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}

class TempCtrl {
public:
	/**
	 * @brief 构造函数。
	 * @param htim PWM 定时器句柄（TIM3）。
	 * @param channel PWM 通道（TIM_CHANNEL_3）。
	 * @param arr 自动重装载值（PWM 分辨率，TIM3 ARR=1000）。
	 */
	TempCtrl(TIM_HandleTypeDef* htim,
			 uint32_t channel,
			 uint32_t arr = 1000U);

	/**
	 * @brief 启动 PWM 输出。
	 * @return true：成功；false：失败。
	 */
	bool Init();

	/**
	 * @brief 设置目标温度。
	 * @param targetC 目标温度，单位 ℃。
	 */
	void SetTarget(float targetC);

	/**
	 * @brief 更新 PID 输出（周期调用，与飞控同频 100Hz）。
	 * @param currentC 当前温度，单位 ℃。
	 * @return 当前 PWM 占空比（0 ~ arr）。
	 */
	uint32_t Update(float currentC);

	/** 当前温度（Watch 窗口查看）。 */
	float current_temp_ = 0.0f;
	/** 目标温度。 */
	float target_temp_ = 0.0f;
	/** PID 输出占空比。 */
	uint32_t pwm_duty_ = 0U;

private:
	TIM_HandleTypeDef* htim_;
	uint32_t channel_;
	uint32_t arr_;

	// PID 参数
	float kp_ = 8.0f;
	float ki_ = 0.0f;
	float kd_ = 0.0f;

	float integral_ = 0.0f;
	float prev_error_ = 0.0f;
	bool first_update_ = true;

	// 积分限幅
	static constexpr float kIntegralLimit = 500.0f;
	// 死区（℃）：误差小于此值时不动作
	static constexpr float kDeadZone = 0.5f;
};

#endif
