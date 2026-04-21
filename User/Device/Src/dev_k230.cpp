/**
 * @file dev_k230.cpp
 * @brief K230 指令解析设备层实现。
 * @details 实现 UART 指令流拆包、校验与制导指令更新。
 */
#include "dev_k230.h"

#include "Config.h"

#include <cmath>
#include <cstring>

#include "drv_uart.h"

namespace {

// 协议帧头第 1 字节。
constexpr uint8_t kHeader0 = 0x5AU;
// 协议帧头第 2 字节。
constexpr uint8_t kHeader1 = 0xA5U;
// 负载长度（2 个 float，共 8 字节）。
constexpr uint8_t kPayloadLength = 8U;
// 帧总长：帧头 2 + 负载 8 + 校验 1。
constexpr uint8_t kFrameLength = 2U + kPayloadLength + 1U;

} // namespace

/**
 * @brief 构造函数。
 */
K230::K230()
	: huart_(nullptr),
	  latest_cmd_{0.0f, 0.0f},
	  has_valid_cmd_(false),
	  frame_buf_{0U},
	  frame_index_(0U)
{
}

/**
 * @brief 初始化 K230 接收链路。
 * @param huart UART 句柄。
 * @param dmaBuffer DMA 接收缓冲区。
 * @param bufferSize 缓冲区长度。
 * @return true：初始化成功；false：初始化失败。
 */
bool K230::Init(UART_HandleTypeDef* huart, uint8_t* dmaBuffer, uint16_t bufferSize)
{
	if ((huart == nullptr) || (dmaBuffer == nullptr) || (bufferSize == 0U)) {
		return false;
	}

	huart_ = huart;
	// 清空帧状态机索引。
	frame_index_ = 0U;

	// 注册 UART 接收回调，让 DrvUART 在空闲接收事件中回调本对象。
	DrvUART::RxEventHandler handler = {};
	handler.huart = huart;
	handler.buffer = dmaBuffer;
	handler.bufferSize = bufferSize;
	handler.callback = &K230::UartRxEventThunk;
	handler.userContext = this;

	if (!DrvUART::RegisterRxEventHandler(handler)) {
		return false;
	}

	DrvUART::StartReceive_DMA_Idle(huart, dmaBuffer, bufferSize);
	return true;
}

/**
 * @brief 协议流式解析函数。
 * @param pBuffer 输入缓冲区。
 * @param len 输入长度。
 * @return 无。
 */
void K230::ParseProtocol(uint8_t* pBuffer, uint16_t len)
{
	if ((pBuffer == nullptr) || (len == 0U)) {
		return;
	}

	for (uint16_t i = 0U; i < len; ++i) {
		uint8_t byte = pBuffer[i];

		// 状态 0：等待第 1 帧头字节。
		if (frame_index_ == 0U) {
			if (byte == kHeader0) {
				frame_buf_[frame_index_++] = byte;
			}
			continue;
		}

		// 状态 1：等待第 2 帧头字节，并支持重同步。
		if (frame_index_ == 1U) {
			if (byte == kHeader1) {
				frame_buf_[frame_index_++] = byte;
			} else if (byte == kHeader0) {
				frame_buf_[0] = byte;
				frame_index_ = 1U;
			} else {
				frame_index_ = 0U;
			}
			continue;
		}

		// 状态 2..N：收集负载与校验，满帧后执行解码。
		frame_buf_[frame_index_++] = byte;
		if (frame_index_ >= kFrameLength) {
			(void)DecodeFrame(frame_buf_);
			frame_index_ = 0U;
		}
	}
}

/**
 * @brief 获取最新有效制导指令。
 * @return GuidanceCmd 当前缓存指令。
 */
GuidanceCmd K230::GetGuidanceCommand() const
{
	return latest_cmd_;
}

/**
 * @brief 查询有效指令标志。
 * @return true：已有有效指令；false：暂无有效指令。
 */
bool K230::HasValidCommand() const
{
	return has_valid_cmd_;
}

/**
 * @brief UART 回调桥接函数。
 * @param huart UART 句柄。
 * @param buffer 数据缓冲区。
 * @param length 数据长度。
 * @param userContext 用户上下文（K230 对象指针）。
 * @return 无。
 */
void K230::UartRxEventThunk(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t length, void* userContext)
{
	K230* self = static_cast<K230*>(userContext);
	if ((self == nullptr) || (huart != self->huart_)) {
		return;
	}

	self->ParseProtocol(buffer, length);
}

/**
 * @brief 计算 8 位和校验。
 * @param pData 数据起始指针。
 * @param length 校验长度。
 * @return uint8_t 校验和低 8 位。
 */
uint8_t K230::CalcChecksum(const uint8_t* pData, uint16_t length)
{
	uint16_t sum = 0U;
	for (uint16_t i = 0U; i < length; ++i) {
		sum += pData[i];
	}

	return static_cast<uint8_t>(sum & 0xFFU);
}

/**
 * @brief 解析并校验完整指令帧。
 * @param frame 帧缓存指针。
 * @return true：帧有效并完成更新；false：帧无效。
 */
bool K230::DecodeFrame(const uint8_t* frame)
{
	if (frame == nullptr) {
		return false;
	}

	// 校验帧头。
	if ((frame[0] != kHeader0) || (frame[1] != kHeader1)) {
		return false;
	}

	// 校验整帧和校验值。
	uint8_t checksum = CalcChecksum(frame, static_cast<uint16_t>(kFrameLength - 1U));
	if (checksum != frame[kFrameLength - 1U]) {
		return false;
	}

	// 负载按小端 float 序列解析 ny、nz。
	float ny = 0.0f;
	float nz = 0.0f;
	std::memcpy(&ny, &frame[2], sizeof(float));
	std::memcpy(&nz, &frame[6], sizeof(float));

	// 过滤 NaN/Inf 异常数据，增强抗干扰能力。
	if (!std::isfinite(ny) || !std::isfinite(nz)) {
		return false;
	}

	// 更新最新有效指令。
	latest_cmd_.ny_c = ny;
	latest_cmd_.nz_c = nz;
	has_valid_cmd_ = true;
	return true;
}
