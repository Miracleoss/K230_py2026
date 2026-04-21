/**
 * @file dev_bmi088.cpp
 * @brief BMI088 设备层实现。
 * @details 完成 BMI088 初始化、原始数据读取与物理量换算。
 */
#include "dev_bmi088.h"

#include "Config.h"

#include "drv_delay.h"
#include "drv_gpio.h"
#include "drv_spi.h"

namespace {

// ===== 加速度计寄存器地址 =====
// 芯片 ID 寄存器地址。
constexpr uint8_t kAccelChipIdReg = 0x00U;
// 期望的加速度计芯片 ID。
constexpr uint8_t kAccelChipId = 0x1EU;
// 加速度数据起始寄存器地址（X_LSB）。
constexpr uint8_t kAccelDataStartReg = 0x12U;
// 加速度带宽/滤波配置寄存器。
constexpr uint8_t kAccelConfReg = 0x40U;
// 加速度量程配置寄存器。
constexpr uint8_t kAccelRangeReg = 0x41U;
// 加速度电源配置寄存器。
constexpr uint8_t kAccelPwrConfReg = 0x7CU;
// 加速度电源控制寄存器。
constexpr uint8_t kAccelPwrCtrlReg = 0x7DU;
// 加速度软复位寄存器。
constexpr uint8_t kAccelSoftResetReg = 0x7EU;

// ===== 陀螺仪寄存器地址 =====
// 芯片 ID 寄存器地址。
constexpr uint8_t kGyroChipIdReg = 0x00U;
// 期望的陀螺仪芯片 ID。
constexpr uint8_t kGyroChipId = 0x0FU;
// 陀螺仪数据起始寄存器地址（X_LSB）。
constexpr uint8_t kGyroDataStartReg = 0x02U;
// 陀螺仪量程配置寄存器。
constexpr uint8_t kGyroRangeReg = 0x0FU;
// 陀螺仪带宽/采样率配置寄存器。
constexpr uint8_t kGyroBandwidthReg = 0x10U;
// 陀螺仪低功耗模式控制寄存器。
constexpr uint8_t kGyroLpm1Reg = 0x11U;

// ===== 配置值 =====
// BMI088 软复位命令。
constexpr uint8_t kSoftResetCmd = 0xB6U;
// 加速度功耗模式：Active。
constexpr uint8_t kAccelPwrConfActive = 0x00U;
// 加速度电源使能。
constexpr uint8_t kAccelPwrCtrlOn = 0x04U;
// 加速度量程：±6g。
constexpr uint8_t kAccelRange6G = 0x01U;
// 加速度带宽配置（来源于统一硬件配置）。
constexpr uint8_t kAccelBandwidthNormal = HwConfig::Bmi088::kAccelBandwidthRegValue;

// 陀螺仪量程：±2000 dps。
constexpr uint8_t kGyroRange2000Dps = 0x00U;
// 陀螺仪带宽/采样率配置（来源于统一硬件配置）。
constexpr uint8_t kGyroBandwidth1000Hz = HwConfig::Bmi088::kGyroBandwidthRegValue;
// 陀螺仪正常工作模式。
constexpr uint8_t kGyroNormalMode = 0x00U;

// ===== 物理量换算常量 =====
// 角度到弧度转换系数。
constexpr float kDegToRad = 0.01745329252f;
// 加速度原始值到 m/s^2 的比例系数。
constexpr float kAccelScaleMps2 = (6.0f * FcParam::Control::kGravity) / 32768.0f;
// 陀螺仪原始值到 rad/s 的比例系数。
constexpr float kGyroScaleRads = (2000.0f * kDegToRad) / 32768.0f;

} // namespace

/**
 * @brief 构造 BMI088 设备对象。
 * @param hspi SPI 句柄。
 * @param accelCsPort 加速度片选端口。
 * @param accelCsPin 加速度片选引脚。
 * @param gyroCsPort 陀螺仪片选端口。
 * @param gyroCsPin 陀螺仪片选引脚。
 */
Bmi088::Bmi088(SPI_HandleTypeDef* hspi,
			   GPIO_TypeDef* accelCsPort,
			   uint16_t accelCsPin,
			   GPIO_TypeDef* gyroCsPort,
			   uint16_t gyroCsPin)
	: hspi_(hspi),
	  accel_cs_port_(accelCsPort),
	  accel_cs_pin_(accelCsPin),
	  gyro_cs_port_(gyroCsPort),
	  gyro_cs_pin_(gyroCsPin)
{
}

/**
 * @brief 初始化 BMI088。
 * @return true：初始化成功；false：初始化失败。
 */
bool Bmi088::Init()
{
	if ((hspi_ == nullptr) || (accel_cs_port_ == nullptr) || (gyro_cs_port_ == nullptr)) {
		return false;
	}

	// 上电后先将两路 CS 置为非选中状态。
	DrvGPIO::SetPin(accel_cs_port_, accel_cs_pin_);
	DrvGPIO::SetPin(gyro_cs_port_, gyro_cs_pin_);

	// 读取并校验加速度计芯片 ID。
	uint8_t id = 0U;
	if ((!ReadAccelRegs(kAccelChipIdReg, &id, 1U)) || (id != kAccelChipId)) {
		return false;
	}

	// 读取并校验陀螺仪芯片 ID。
	if ((!ReadGyroRegs(kGyroChipIdReg, &id, 1U)) || (id != kGyroChipId)) {
		return false;
	}

	// 加速度初始化：软复位 -> 等待 -> 进入 Active -> 设置量程与带宽。
	if (!WriteAccelReg(kAccelSoftResetReg, kSoftResetCmd)) {
		return false;
	}
	// 官方建议复位后等待芯片稳定。
	Delay_ms(50U);

	if (!WriteAccelReg(kAccelPwrConfReg, kAccelPwrConfActive)) {
		return false;
	}
	Delay_ms(2U);

	if (!WriteAccelReg(kAccelPwrCtrlReg, kAccelPwrCtrlOn)) {
		return false;
	}
	Delay_ms(2U);

	if (!WriteAccelReg(kAccelRangeReg, kAccelRange6G)) {
		return false;
	}

	if (!WriteAccelReg(kAccelConfReg, kAccelBandwidthNormal)) {
		return false;
	}

	// 陀螺仪初始化：正常模式 + 量程 + 采样率。
	if (!WriteGyroReg(kGyroLpm1Reg, kGyroNormalMode)) {
		return false;
	}

	if (!WriteGyroReg(kGyroRangeReg, kGyroRange2000Dps)) {
		return false;
	}

	if (!WriteGyroReg(kGyroBandwidthReg, kGyroBandwidth1000Hz)) {
		return false;
	}

	return true;
}

/**
 * @brief 读取并换算传感器数据。
 * @param outData 输出数据结构体。
 * @return true：读取成功；false：读取失败。
 */
bool Bmi088::ReadSensor(Data* outData)
{
	if (outData == nullptr) {
		return false;
	}

	// 读取加速度/陀螺仪 3 轴原始字节流。
	uint8_t accel_buf[6] = {0U};
	uint8_t gyro_buf[6] = {0U};

	if (!ReadAccelRegs(kAccelDataStartReg, accel_buf, 6U)) {
		return false;
	}

	if (!ReadGyroRegs(kGyroDataStartReg, gyro_buf, 6U)) {
		return false;
	}

	// 字节流拼接为 16 位有符号原始值。
	outData->raw.accel_x = ToInt16(accel_buf[0], accel_buf[1]);
	outData->raw.accel_y = ToInt16(accel_buf[2], accel_buf[3]);
	outData->raw.accel_z = ToInt16(accel_buf[4], accel_buf[5]);

	outData->raw.gyro_x = ToInt16(gyro_buf[0], gyro_buf[1]);
	outData->raw.gyro_y = ToInt16(gyro_buf[2], gyro_buf[3]);
	outData->raw.gyro_z = ToInt16(gyro_buf[4], gyro_buf[5]);

	// 原始计数值转换为物理单位。
	outData->accel_mps2[0] = static_cast<float>(outData->raw.accel_x) * kAccelScaleMps2;
	outData->accel_mps2[1] = static_cast<float>(outData->raw.accel_y) * kAccelScaleMps2;
	outData->accel_mps2[2] = static_cast<float>(outData->raw.accel_z) * kAccelScaleMps2;

	outData->gyro_rads[0] = static_cast<float>(outData->raw.gyro_x) * kGyroScaleRads;
	outData->gyro_rads[1] = static_cast<float>(outData->raw.gyro_y) * kGyroScaleRads;
	outData->gyro_rads[2] = static_cast<float>(outData->raw.gyro_z) * kGyroScaleRads;

	return true;
}

/**
 * @brief 写加速度计单寄存器。
 * @param regAddr 寄存器地址。
 * @param data 写入数据。
 * @return true：成功；false：失败。
 */
bool Bmi088::WriteAccelReg(uint8_t regAddr, uint8_t data)
{
	return DrvSPI::WriteRegister(hspi_, accel_cs_port_, accel_cs_pin_, regAddr, data);
}

/**
 * @brief 读加速度计连续寄存器。
 * @param regAddr 起始寄存器地址。
 * @param pData 输出缓冲区。
 * @param size 读取长度。
 * @return true：成功；false：失败。
 */
bool Bmi088::ReadAccelRegs(uint8_t regAddr, uint8_t* pData, uint16_t size)
{
	return DrvSPI::ReadRegisters(hspi_, accel_cs_port_, accel_cs_pin_, regAddr, pData, size);
}

/**
 * @brief 写陀螺仪单寄存器。
 * @param regAddr 寄存器地址。
 * @param data 写入数据。
 * @return true：成功；false：失败。
 */
bool Bmi088::WriteGyroReg(uint8_t regAddr, uint8_t data)
{
	return DrvSPI::WriteRegister(hspi_, gyro_cs_port_, gyro_cs_pin_, regAddr, data);
}

/**
 * @brief 读陀螺仪连续寄存器。
 * @param regAddr 起始寄存器地址。
 * @param pData 输出缓冲区。
 * @param size 读取长度。
 * @return true：成功；false：失败。
 */
bool Bmi088::ReadGyroRegs(uint8_t regAddr, uint8_t* pData, uint16_t size)
{
	return DrvSPI::ReadRegisters(hspi_, gyro_cs_port_, gyro_cs_pin_, regAddr, pData, size);
}

/**
 * @brief 低高字节拼接为 int16_t。
 * @param lsb 低字节。
 * @param msb 高字节。
 * @return int16_t 拼接结果。
 */
int16_t Bmi088::ToInt16(uint8_t lsb, uint8_t msb)
{
	return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8U) | static_cast<uint16_t>(lsb));
}
