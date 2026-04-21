/**
 * @file dev_k230.h
 * @brief K230 视觉指令设备层接口。
 * @details 负责 UART 指令流解析，输出制导法向过载指令。
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
 * @brief 制导指令数据结构。
 */
struct GuidanceCmd {
	/** Y 方向法向过载指令。 */
	float ny_c;
	/** Z 方向法向过载指令。 */
	float nz_c;
};

/**
 * @brief K230 协议解析类。
 */
class K230 {
public:
	/**
	 * @brief 构造函数。
	 */
	K230();

	/**
	 * @brief 绑定 UART DMA+空闲接收并启动数据链路。
	 * @param huart K230 使用的 UART 句柄。
	 * @param dmaBuffer DMA 接收缓冲区。
	 * @param bufferSize 缓冲区大小。
	 * @return true：初始化成功；false：初始化失败。
	 */
	bool Init(UART_HandleTypeDef* huart, uint8_t* dmaBuffer, uint16_t bufferSize);

	/**
	 * @brief 解析来自 DrvUART 回调的数据块。
	 * @param pBuffer 数据指针。
	 * @param len 数据长度。
	 * @return 无。
	 */
	void ParseProtocol(uint8_t* pBuffer, uint16_t len);

	/**
	 * @brief 获取最新有效制导指令。
	 * @return GuidanceCmd 最新指令。
	 */
	GuidanceCmd GetGuidanceCommand() const;

	/**
	 * @brief 查询是否收到过有效指令帧。
	 * @return true：已收到有效帧；false：尚未收到。
	 */
	bool HasValidCommand() const;

private:
	/** UART 回调静态桥接函数。 */
	static void UartRxEventThunk(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t length, void* userContext);
	/** 计算帧校验和。 */
	static uint8_t CalcChecksum(const uint8_t* pData, uint16_t length);
	/** 解析并校验完整帧。 */
	bool DecodeFrame(const uint8_t* frame);

	/** 绑定的 UART 句柄。 */
	UART_HandleTypeDef* huart_;
	/** 最新有效制导指令。 */
	GuidanceCmd latest_cmd_;
	/** 是否存在有效指令标志。 */
	bool has_valid_cmd_;

	/** 帧解析缓存（2 字节帧头 + 8 字节负载 + 1 字节校验）。 */
	uint8_t frame_buf_[11];
	/** 当前已缓存字节数。 */
	uint8_t frame_index_;
};

#endif
