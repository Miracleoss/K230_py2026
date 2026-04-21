/**
 * @file drv_uart.h
 * @brief UART 驱动层接口。
 * @details 提供 DMA+空闲中断接收、阻塞发送、回调注册与事件分发能力。
 */
#pragma once

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdbool.h>
#include <stdint.h>
#endif

#include "main.h"

/**
 * @brief C 语言桥接入口。
 * @details 在 main.c 的 HAL_UARTEx_RxEventCallback 中调用，将事件转发到 C++ 驱动层。
 * @param huart 触发事件的 UART 句柄。
 * @param size 本次接收有效字节数。
 * @return 无。
 */
void DrvUART_RxEventIRQHandler(UART_HandleTypeDef* huart, uint16_t size);

#ifdef __cplusplus
}

namespace DrvUART {

/**
 * @brief DMA+空闲中断接收事件回调类型。
 * @param huart UART 句柄。
 * @param buffer 接收缓冲区指针。
 * @param length 本次有效数据长度。
 * @param userContext 用户上下文指针。
 * @return 无。
 */
using RxEventCallback = void (*)(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t length, void* userContext);

/**
 * @brief UART 接收事件处理项。
 */
struct RxEventHandler {
	/** 绑定的 UART 实例。 */
	UART_HandleTypeDef* huart;
	/** DMA 接收缓冲区地址。 */
	uint8_t* buffer;
	/** DMA 接收缓冲区大小（字节）。 */
	uint16_t bufferSize;
	/** 收到 RxEvent 时调用的回调函数。 */
	RxEventCallback callback;
	/** 用户私有上下文指针。 */
	void* userContext;
};

/**
 * @brief 启动 UART DMA 空闲接收模式。
 * @param huart UART 句柄。
 * @param buffer DMA 接收缓冲区。
 * @param size 缓冲区长度。
 * @return 无。
 */
void StartReceive_DMA_Idle(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t size);

/**
 * @brief 阻塞式 UART 发送。
 * @param huart UART 句柄。
 * @param pData 发送数据指针。
 * @param size 发送长度。
 * @param timeout 超时时间，单位 ms。
 * @return true：发送成功；false：发送失败或参数无效。
 */
bool Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t size, uint32_t timeout = 100U);

/**
 * @brief 注册或更新一个 UART 接收事件处理项。
 * @param handler 待注册处理项。
 * @return true：注册成功；false：注册失败。
 */
bool RegisterRxEventHandler(const RxEventHandler& handler);

/**
 * @brief 反注册 UART 接收事件处理项。
 * @param huart 目标 UART 句柄。
 * @return 无。
 */
void UnregisterRxEventHandler(UART_HandleTypeDef* huart);

/**
 * @brief 分发一次接收事件并重启 DMA 空闲接收。
 * @param huart UART 句柄。
 * @param size 本次有效数据长度。
 * @return 无。
 */
void HandleRxEvent(UART_HandleTypeDef* huart, uint16_t size);

} // namespace DrvUART
#endif
