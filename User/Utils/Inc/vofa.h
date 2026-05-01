#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

//#ifdef __cplusplus
}

#ifdef __cplusplus

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace Vofa {

static UART_HandleTypeDef* g_huart = nullptr;

inline void Init(UART_HandleTypeDef* huart)
{
	g_huart = huart;
}

// FireWater: printf 格式，末尾自动加 \r\n
inline void FireWater(const char* format, ...)
{
	if (g_huart == nullptr) return;

	uint8_t txBuffer[100];
	va_list args;
	va_start(args, format);
	uint32_t n = vsnprintf(reinterpret_cast<char*>(txBuffer), sizeof(txBuffer), format, args);
	va_end(args);

	HAL_UART_Transmit_DMA(g_huart, txBuffer, n);
}

// JustFloat: 二进制协议，高效高频输出
inline void JustFloat(const float* data, uint8_t num)
{
	if (g_huart == nullptr) return;

	uint8_t tempData[100];
	const uint8_t temp_end[4] = {0x00, 0x00, 0x80, 0x7F};

	memcpy(tempData, data, sizeof(float) * num);
	memcpy(&tempData[num * 4], temp_end, 4);

	HAL_UART_Transmit_DMA(g_huart, tempData, (num + 1) * 4);
}

} // namespace Vofa

#endif
