/**
 * @file drv_spi.cpp
 * @brief SPI 驱动实现。
 * @details 提供通用全双工传输与寄存器读写，内部负责片选控制。
 */
#include "drv_spi.h"
#include "Config.h"

#include "drv_gpio.h"

// SPI 读寄存器地址位掩码（bit7=1 表示读）。
constexpr uint8_t kReadMask = 0x80U;
// SPI 写寄存器地址位掩码（bit7=0 表示写）。
constexpr uint8_t kWriteMask = 0x7FU;
// 默认 SPI 事务超时时间，单位 ms。
constexpr uint32_t kSpiTimeoutMs = 10U;


namespace DrvSPI 
{
	/**
	 * @brief SPI 全双工收发实现。
	 * @param hspi SPI 句柄。
	 * @param txData 发送缓冲区。
	 * @param rxData 接收缓冲区。
	 * @param size 收发长度。
	 * @param timeout 超时时间（ms）。
	 * @return true：成功；false：失败。
	 */
	bool TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* txData, uint8_t* rxData, uint16_t size, uint32_t timeout)
	{
		if ((hspi == nullptr) || (txData == nullptr) || (rxData == nullptr) || (size == 0U)) {
			return false;
		}

		return (HAL_SPI_TransmitReceive(hspi, txData, rxData, size, timeout) == HAL_OK);
	}

	/**
	 * @brief 连续读取寄存器实现。
	 * @param hspi SPI 句柄。
	 * @param csPort 片选端口。
	 * @param csPin 片选引脚。
	 * @param regAddr 起始寄存器地址。
	 * @param pData 数据输出缓冲区。
	 * @param size 读取长度。
	 * @return true：成功；false：失败。
	 */
	bool ReadRegisters(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin, uint8_t regAddr, uint8_t* pData, uint16_t size)
	{
		if ((hspi == nullptr) || (csPort == nullptr) || (pData == nullptr) || (size == 0U)) {
			return false;
		}

		bool result = true;
		// 读操作地址阶段：地址 OR 读位。
		uint8_t tx_addr = static_cast<uint8_t>(regAddr | kReadMask);
		uint8_t rx_addr = 0U;

		// 拉低片选，开始 SPI 帧事务。
		DrvGPIO::ResetPin(csPort, csPin);

		result = TransmitReceive(hspi, &tx_addr, &rx_addr, 1U, kSpiTimeoutMs);
		if (result) {
			for (uint16_t i = 0U; i < size; ++i) {
				// 发送 dummy 字节产生时钟，从从设备移出 1 字节数据。
				uint8_t tx_dummy = 0xFFU;
				uint8_t rx_byte = 0U;

				result = TransmitReceive(hspi, &tx_dummy, &rx_byte, 1U, kSpiTimeoutMs);
				if (!result) {
					break;
				}

				pData[i] = rx_byte;
			}
		}

		// 释放片选，结束 SPI 帧事务。
		DrvGPIO::SetPin(csPort, csPin);
		return result;
	}

	/**
	 * @brief 单寄存器写入实现。
	 * @param hspi SPI 句柄。
	 * @param csPort 片选端口。
	 * @param csPin 片选引脚。
	 * @param regAddr 寄存器地址。
	 * @param data 写入数据。
	 * @return true：成功；false：失败。
	 */
	bool WriteRegister(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin, uint8_t regAddr, uint8_t data)
	{
		if ((hspi == nullptr) || (csPort == nullptr)) {
			return false;
		}

		// 写操作发送 2 字节：寄存器地址 + 数据。
		uint8_t tx_buf[2] = {static_cast<uint8_t>(regAddr & kWriteMask), data};
		// 读缓冲仅用于占位以完成全双工时序。
		uint8_t rx_buf[2] = {0U, 0U};

		DrvGPIO::ResetPin(csPort, csPin);
		bool result = TransmitReceive(hspi, tx_buf, rx_buf, 2U, kSpiTimeoutMs);
		DrvGPIO::SetPin(csPort, csPin);

		return result;
	}

} // namespace DrvSPI
