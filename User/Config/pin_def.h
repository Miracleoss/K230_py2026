#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}

namespace PinDef {

struct GpioPin {
	GPIO_TypeDef* port;
	uint16_t pin;
};

namespace BMI088 {

// SPI1: SCK=PA5, MISO=PA6, MOSI=PA7 (由 CubeMX 外设配置管理)
static const GpioPin kAccelCs = {GPIOA, GPIO_PIN_2};
static const GpioPin kGyroCs = {GPIOA, GPIO_PIN_3};
static const GpioPin kInt = {GPIOA, GPIO_PIN_4};

} // namespace BMI088

namespace K230D {

// UART1 重映射: TX=PB6, RX=PB7 (由 CubeMX 外设配置管理)
static const GpioPin kUart1Tx = {GPIOB, GPIO_PIN_6};
static const GpioPin kUart1Rx = {GPIOB, GPIO_PIN_7};

} // namespace K230D

namespace Servo {

// TIM1 CH1~CH4 输出引脚
static const GpioPin kCh1 = {GPIOA, GPIO_PIN_8};
static const GpioPin kCh2 = {GPIOA, GPIO_PIN_9};
static const GpioPin kCh3 = {GPIOA, GPIO_PIN_10};
static const GpioPin kCh4 = {GPIOA, GPIO_PIN_11};

// TIM1 CH1~CH4 对应通道常量
static constexpr uint32_t SERVO_CH1 = TIM_CHANNEL_1;
static constexpr uint32_t SERVO_CH2 = TIM_CHANNEL_2;
static constexpr uint32_t SERVO_CH3 = TIM_CHANNEL_3;
static constexpr uint32_t SERVO_CH4 = TIM_CHANNEL_4;

} // namespace Servo

namespace Motor {

// 默认电机输出通道（当前工程使用 TIM3_CH3）
static constexpr uint32_t DEFAULT_PWM_CHANNEL = TIM_CHANNEL_3;

} // namespace Motor

namespace Debug {

static const GpioPin kLed = {GPIOC, GPIO_PIN_13};

} // namespace Debug

} // namespace PinDef

#endif

