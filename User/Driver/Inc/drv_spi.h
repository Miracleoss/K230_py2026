/**
 * @file drv_spi.h
 * @brief SPI 驱动层接口。
 * @details 提供通用收发和寄存器读写辅助函数，适配 BMI088 等 SPI 外设。
 */
#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}
#endif

namespace DrvSPI {

/**
 * @brief SPI 全双工收发。
 * @param hspi SPI 句柄。
 * @param txData 发送缓冲区指针。
 * @param rxData 接收缓冲区指针。
 * @param size 收发字节数。
 * @param timeout 超时时间，单位 ms。
 * @return true：传输成功；false：参数错误或 HAL 返回失败。
 */
bool TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* txData, uint8_t* rxData, uint16_t size, uint32_t timeout = 10U);

/**
 * @brief 读取 SPI 设备连续寄存器。
 * @details 软件控制片选，读格式为：地址（读位）+ dummy 时钟读取多字节。
 * @param hspi SPI 句柄。
 * @param csPort 片选 GPIO 端口。
 * @param csPin 片选 GPIO 引脚。
 * @param regAddr 起始寄存器地址。
 * @param pData 数据输出缓冲区。
 * @param size 读取字节数。
 * @return true：读取成功；false：读取失败。
 */
bool ReadRegisters(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin, uint8_t regAddr, uint8_t* pData, uint16_t size);

/**
 * @brief 写 SPI 设备单个寄存器。
 * @param hspi SPI 句柄。
 * @param csPort 片选 GPIO 端口。
 * @param csPin 片选 GPIO 引脚。
 * @param regAddr 目标寄存器地址。
 * @param data 待写入 1 字节数据。
 * @return true：写入成功；false：写入失败。
 */
bool WriteRegister(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin, uint8_t regAddr, uint8_t data);

} // namespace DrvSPI
