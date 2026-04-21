/**
 * @file dev_motor.h
 * @brief 电机设备层接口。
 * @details 提供油门抽象、占空比映射以及急停控制。
 */
#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}

#include "Config.h"

/**
 * @brief 电机 PWM 控制类。
 */
class Motor {
public:
	/**
	 * @brief 构造函数。
	 * @param htim PWM 定时器句柄。
	 * @param channel PWM 输出通道。
	 */
	explicit Motor(TIM_HandleTypeDef* htim = nullptr, uint32_t channel = PinDef::Motor::DEFAULT_PWM_CHANNEL);

	/**
	 * @brief 初始化电机 PWM 输出。
	 * @return true：初始化成功；false：初始化失败。
	 */
	bool Init();

	/**
	 * @brief 设置油门。
	 * @param duty 油门占空比，范围 [0.0, 1.0]。
	 * @return 无。
	 */
	void SetThrottle(float duty);

	/**
	 * @brief 急停，直接将 PWM 输出置零。
	 * @return 无。
	 */
	void EmergencyStop();

	/**
	 * @brief 绑定输出定时器与通道。
	 * @param htim PWM 定时器句柄。
	 * @param channel PWM 通道。
	 * @return 无。
	 */
	void BindOutput(TIM_HandleTypeDef* htim, uint32_t channel);

	/**
	 * @brief 获取当前油门值。
	 * @return float 当前油门占空比。
	 */
	float GetThrottle() const;

private:
	/** 油门输入限幅到 [0.0, 1.0]。 */
	static float ClampDuty(float duty);
	/** 将占空比映射到比较值。 */
	static uint32_t DutyToCompare(TIM_HandleTypeDef* htim, float duty);

	/** PWM 定时器句柄。 */
	TIM_HandleTypeDef* htim_;
	/** PWM 通道。 */
	uint32_t channel_;
	/** 当前油门占空比缓存。 */
	float throttle_;
};

#endif
