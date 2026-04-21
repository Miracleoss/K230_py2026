#pragma once

#ifdef __cplusplus

#include <cstdint>

namespace HwConfig {

static constexpr uint32_t kSystemClockHz = 72000000U;

namespace Tim1 {

static constexpr uint32_t kPwmFrequencyHz = 50U;
static constexpr uint32_t kPrescaler = 720U;
static constexpr uint32_t kAutoReload = 2000U;

} // namespace Tim1

namespace Uart1 {

static constexpr uint32_t kBaudRate = 115200U;

} // namespace Uart1

namespace Bmi088 {

static constexpr uint8_t kAccelBandwidthRegValue = 0xA8U;
static constexpr uint8_t kGyroBandwidthRegValue = 0x02U;

} // namespace Bmi088

namespace Task {

static constexpr uint32_t kSchedulerHz = 100U;
static constexpr uint32_t kTickMs = 10U;
static constexpr uint8_t kTask20MsDivider = 2U;

} // namespace Task

namespace App {

static constexpr uint32_t kFlightPeriodMs = 10U;
static constexpr uint32_t kK230TimeoutMs = 150U;
static constexpr uint16_t kK230RxBufferSize = 64U;

} // namespace App

} // namespace HwConfig

#endif

