/**
 * @file drv_spi.cpp
 * @brief SPI 驱动实现。
 * @details 提供通用全双工传输与 BMI088 加速度计/陀螺仪专用寄存器读写。
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

	bool TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* txData, uint8_t* rxData, uint16_t size, uint32_t timeout)
	{
		if ((hspi == nullptr) || (txData == nullptr) || (rxData == nullptr) || (size == 0U)) {
			return false;
		}

		return (HAL_SPI_TransmitReceive(hspi, txData, rxData, size, timeout) == HAL_OK);
	}

	bool AccelReadSingle(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin,
						 uint8_t regAddr, uint8_t* pData)
	{
		if ((hspi == nullptr) || (csPort == nullptr) || (pData == nullptr)) {
			return false;
		}

		bool result = true;
		uint8_t tx_addr = static_cast<uint8_t>(regAddr | kReadMask);
		uint8_t rx_dummy = 0U;
		uint8_t tx_dummy = 0x55U;
		uint8_t rx_data = 0U;

		DrvGPIO::ResetPin(csPort, csPin);

		// 帧1: 地址字节。
		result = TransmitReceive(hspi, &tx_addr, &rx_dummy, 1U, kSpiTimeoutMs);
		if (result) {
			// 帧2: dummy 字节（MISO 无效，必须丢弃）。
			result = TransmitReceive(hspi, &tx_dummy, &rx_dummy, 1U, kSpiTimeoutMs);
		}
		if (result) {
			// 帧3: 读取数据。
			result = TransmitReceive(hspi, &tx_dummy, &rx_data, 1U, kSpiTimeoutMs);
			if (result) {
				*pData = rx_data;
			}
		}

		DrvGPIO::SetPin(csPort, csPin);
		return result;
	}

	bool AccelReadMulti(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin,
						uint8_t regAddr, uint8_t* pData, uint16_t size)
	{
		if ((hspi == nullptr) || (csPort == nullptr) || (pData == nullptr) || (size == 0U)) {
			return false;
		}

		bool result = true;
		uint8_t tx_addr = static_cast<uint8_t>(regAddr | kReadMask);
		uint8_t rx_dummy = 0U;
		uint8_t tx_dummy = 0x55U;

		DrvGPIO::ResetPin(csPort, csPin);

		// 帧1: 地址字节。
		result = TransmitReceive(hspi, &tx_addr, &rx_dummy, 1U, kSpiTimeoutMs);
		if (result) {
			// 帧2: 第二地址字节（dummy，MISO 无效）。
			result = TransmitReceive(hspi, &tx_addr, &rx_dummy, 1U, kSpiTimeoutMs);
		}
		if (result) {
			// 帧3..N+2: 读取数据。
			for (uint16_t i = 0U; i < size; ++i) {
				uint8_t rx_byte = 0U;
				result = TransmitReceive(hspi, &tx_dummy, &rx_byte, 1U, kSpiTimeoutMs);
				if (!result) {
					break;
				}
				pData[i] = rx_byte;
			}
		}

		DrvGPIO::SetPin(csPort, csPin);
		return result;
	}

	bool GyroReadSingle(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin,
						uint8_t regAddr, uint8_t* pData)
	{
		if ((hspi == nullptr) || (csPort == nullptr) || (pData == nullptr)) {
			return false;
		}

		bool result = true;
		uint8_t tx_addr = static_cast<uint8_t>(regAddr | kReadMask);
		uint8_t rx_dummy = 0U;
		uint8_t tx_dummy = 0x55U;
		uint8_t rx_data = 0U;

		DrvGPIO::ResetPin(csPort, csPin);

		// 帧1: 地址字节。
		result = TransmitReceive(hspi, &tx_addr, &rx_dummy, 1U, kSpiTimeoutMs);
		if (result) {
			// 帧2: 读取数据。
			result = TransmitReceive(hspi, &tx_dummy, &rx_data, 1U, kSpiTimeoutMs);
			if (result) {
				*pData = rx_data;
			}
		}

		DrvGPIO::SetPin(csPort, csPin);
		return result;
	}

	bool GyroReadMulti(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin,
					   uint8_t regAddr, uint8_t* pData, uint16_t size)
	{
		if ((hspi == nullptr) || (csPort == nullptr) || (pData == nullptr) || (size == 0U)) {
			return false;
		}

		bool result = true;
		uint8_t tx_addr = static_cast<uint8_t>(regAddr | kReadMask);
		uint8_t rx_dummy = 0U;
		uint8_t tx_dummy = 0x55U;

		DrvGPIO::ResetPin(csPort, csPin);

		// 帧1: 地址字节。
		result = TransmitReceive(hspi, &tx_addr, &rx_dummy, 1U, kSpiTimeoutMs);
		if (result) {
			// 帧2..N+1: 读取数据。
			for (uint16_t i = 0U; i < size; ++i) {
				uint8_t rx_byte = 0U;
				result = TransmitReceive(hspi, &tx_dummy, &rx_byte, 1U, kSpiTimeoutMs);
				if (!result) {
					break;
				}
				pData[i] = rx_byte;
			}
		}

		DrvGPIO::SetPin(csPort, csPin);
		return result;
	}

	bool WriteRegister(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin, uint8_t regAddr, uint8_t data)
	{
		if ((hspi == nullptr) || (csPort == nullptr)) {
			return false;
		}

		uint8_t tx_buf[2] = {static_cast<uint8_t>(regAddr & kWriteMask), data};
		uint8_t rx_buf[2] = {0U, 0U};

		DrvGPIO::ResetPin(csPort, csPin);
		bool result = TransmitReceive(hspi, tx_buf, rx_buf, 2U, kSpiTimeoutMs);
		DrvGPIO::SetPin(csPort, csPin);

		return result;
	}

} // namespace DrvSPI
