/**
 * @file dev_servo.h
 * @brief 舵机设备层接口。
 * @details 提供 4 路舵偏角到 PWM 比较值的映射与输出能力。
 */
#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}

/**
 * @brief 四舵面控制类。
 */
class Servo {
public:
	/**
	 * @brief 构造函数。
	 * @param htim PWM 定时器句柄，默认使用 TIM1。
	 */
	explicit Servo(TIM_HandleTypeDef* htim = nullptr);

	/**
	 * @brief 启动 CH1~CH4 PWM 输出并写入中位角。
	 * @return true：初始化成功；false：初始化失败。
	 */
	bool Init();

	/**
	 * @brief 设置 4 个舵偏角（单位：度）。
	 * @details 输出前会进行 ±25° 硬限幅保护。
	 * @param delta 4 路舵偏角数组。
	 * @return 无。
	 */
	void SetFinAngles(float delta[4]);

	/**
	 * @brief 运行时绑定定时器句柄。
	 * @param htim PWM 定时器句柄。
	 * @return 无。
	 */
	void BindTimer(TIM_HandleTypeDef* htim);

private:
	/** 对舵偏角做硬限幅。 */
	static float ClampAngleDeg(float angle_deg);
	/** 将舵偏角映射为舵机脉宽（us）。 */
	static float AngleDegToPulseUs(float angle_deg);
	/** 将脉宽（us）映射为定时器比较值。 */
	static uint32_t PulseUsToCompare(TIM_HandleTypeDef* htim, float pulse_us);
	/** 获取定时器输入时钟频率。 */
	static uint32_t GetTimerClockHz(TIM_TypeDef* instance);

	/** PWM 定时器句柄。 */
	TIM_HandleTypeDef* htim_;
};

#endif
