/**
 * @file drv_spi.h
 * @brief SPI 驱动层接口。
 * @details 提供通用收发和 BMI088 加速度计/陀螺仪专用寄存器读写函数。
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
 * @brief 读取加速度计单个寄存器。
 * @details BMI088 加速度计单寄存器读时序（3 帧）：
 *          帧1: regAddr|0x80（地址+读位）
 *          帧2: 0x55（dummy，MISO 无效）
 *          帧3: 0x55（MISO 携带有效数据）
 */
bool AccelReadSingle(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin,
                     uint8_t regAddr, uint8_t* pData);

/**
 * @brief 读取加速度计连续寄存器。
 * @details BMI088 加速度计多字节读时序（N+2 帧）：
 *          帧1: regAddr|0x80（地址+读位）
 *          帧2: regAddr|0x80（第二地址字节，MISO 无效）
 *          帧3..N+2: 0x55（MISO 携带有效数据）
 */
bool AccelReadMulti(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin,
                    uint8_t regAddr, uint8_t* pData, uint16_t size);

/**
 * @brief 读取陀螺仪单个寄存器。
 * @details BMI088 陀螺仪单寄存器读时序（2 帧）：
 *          帧1: regAddr|0x80（地址+读位）
 *          帧2: 0x55（MISO 携带有效数据）
 */
bool GyroReadSingle(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin,
                    uint8_t regAddr, uint8_t* pData);

/**
 * @brief 读取陀螺仪连续寄存器。
 * @details BMI088 陀螺仪多字节读时序（N+1 帧）：
 *          帧1: regAddr|0x80（地址+读位）
 *          帧2..N+1: 0x55（MISO 携带有效数据）
 */
bool GyroReadMulti(SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin,
                   uint8_t regAddr, uint8_t* pData, uint16_t size);

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
