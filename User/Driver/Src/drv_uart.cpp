/**
 * @file drv_uart.cpp
 * @brief UART 驱动实现。
 * @details 负责 DMA+空闲接收的事件注册、分发与自动重启，以及阻塞发送。
 */
#include "drv_uart.h"
#include "Config.h"

namespace {

// 最大可注册 UART 接收处理项数量。
constexpr uint8_t kMaxRxEventHandlers = 4U;

// 静态处理表，避免驱动层使用动态内存。
DrvUART::RxEventHandler g_rx_handlers[kMaxRxEventHandlers] = {};

/**
 * @brief 查找 UART 对应处理项索引。
 * @param huart UART 句柄。
 * @return 索引值；未找到返回 -1。
 */
int32_t FindHandlerIndex(UART_HandleTypeDef* huart)
{
	for (uint8_t i = 0U; i < kMaxRxEventHandlers; ++i) {
		if (g_rx_handlers[i].huart == huart) {
			return static_cast<int32_t>(i);
		}
	}

	return -1;
}

/**
 * @brief 查找空闲处理项槽位。
 * @return 空槽索引；无空槽返回 -1。
 */
int32_t FindFreeHandlerIndex()
{
	for (uint8_t i = 0U; i < kMaxRxEventHandlers; ++i) {
		if (g_rx_handlers[i].huart == nullptr) {
			return static_cast<int32_t>(i);
		}
	}

	return -1;
}

/**
 * @brief 禁用 DMA 半传输中断。
 * @details 该驱动基于空闲中断收包，禁用半传输可减少无效中断负担。
 * @param huart UART 句柄。
 * @return 无。
 */
void DisableDmaHalfTransfer(UART_HandleTypeDef* huart)
{
	if ((huart != nullptr) && (huart->hdmarx != nullptr)) {
		// 接收空闲帧只需要 RxEvent 回调；禁用半传输中断。
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	}
}

} // namespace

namespace DrvUART {

/**
 * @brief 启动 UART DMA 空闲接收。
 * @param huart UART 句柄。
 * @param buffer DMA 缓冲区。
 * @param size 缓冲区长度。
 * @return 无。
 */
void StartReceive_DMA_Idle(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t size)
{
	if ((huart == nullptr) || (buffer == nullptr) || (size == 0U)) {
		return;
	}

	(void)HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, size);
	DisableDmaHalfTransfer(huart);

	// 若该 UART 已注册处理项，同步更新其最新缓冲区信息。
	int32_t index = FindHandlerIndex(huart);
	if (index >= 0) {
		g_rx_handlers[index].buffer = buffer;
		g_rx_handlers[index].bufferSize = size;
	}
}

/**
 * @brief 阻塞式 UART 发送。
 * @param huart UART 句柄。
 * @param pData 数据指针。
 * @param size 数据长度。
 * @param timeout 超时时间（ms）。
 * @return true：发送成功；false：发送失败。
 */
bool Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t size, uint32_t timeout)
{
	if (huart == nullptr) {
		return false;
	}

	if (size == 0U) {
		return true;
	}

	if (pData == nullptr) {
		return false;
	}

	return (HAL_UART_Transmit(huart, const_cast<uint8_t*>(pData), size, timeout) == HAL_OK);
}

/**
 * @brief 注册或更新 UART 接收事件处理项。
 * @param handler 处理项配置。
 * @return true：注册成功；false：注册失败。
 */
bool RegisterRxEventHandler(const RxEventHandler& handler)
{
	if ((handler.huart == nullptr) || (handler.buffer == nullptr) || (handler.bufferSize == 0U) || (handler.callback == nullptr)) {
		return false;
	}

	int32_t index = FindHandlerIndex(handler.huart);
	if (index < 0) {
		index = FindFreeHandlerIndex();
		if (index < 0) {
			return false;
		}
	}

	g_rx_handlers[index] = handler;
	return true;
}

/**
 * @brief 反注册 UART 接收事件处理项。
 * @param huart UART 句柄。
 * @return 无。
 */
void UnregisterRxEventHandler(UART_HandleTypeDef* huart)
{
	if (huart == nullptr) {
		return;
	}

	int32_t index = FindHandlerIndex(huart);
	if (index >= 0) {
		g_rx_handlers[index] = {};
	}
}

/**
 * @brief 处理接收事件并重启 DMA 空闲接收。
 * @param huart UART 句柄。
 * @param size 本次接收长度。
 * @return 无。
 */
void HandleRxEvent(UART_HandleTypeDef* huart, uint16_t size)
{
	if (huart == nullptr) {
		return;
	}

	int32_t index = FindHandlerIndex(huart);
	if (index < 0) {
		return;
	}

	RxEventHandler& handler = g_rx_handlers[index];
	if ((handler.buffer == nullptr) || (handler.bufferSize == 0U)) {
		return;
	}

	// 将长度限制在缓冲区范围内，防止越界访问。
	uint16_t valid_length = (size <= handler.bufferSize) ? size : handler.bufferSize;
	// 仅在存在有效回调且长度非零时分发数据。
	if ((handler.callback != nullptr) && (valid_length > 0U)) {
		handler.callback(huart, handler.buffer, valid_length, handler.userContext);
	}

	// 为下一个可变长度帧重新开启 DMA 空闲接收。
	StartReceive_DMA_Idle(huart, handler.buffer, handler.bufferSize);
}

} // namespace DrvUART

/**
 * @brief C 桥接函数实现。
 * @param huart UART 句柄。
 * @param size 本次接收长度。
 * @return 无。
 */
extern "C" void DrvUART_RxEventIRQHandler(UART_HandleTypeDef* huart, uint16_t size)
{
	DrvUART::HandleRxEvent(huart, size);
}
