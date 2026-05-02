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
constexpr uint8_t kAccelChipIdReg = 0x00U;
constexpr uint8_t kAccelChipId = 0x1EU;
constexpr uint8_t kAccelDataStartReg = 0x12U;
constexpr uint8_t kAccelConfReg = 0x40U;
constexpr uint8_t kAccelRangeReg = 0x41U;
constexpr uint8_t kAccelPwrConfReg = 0x7CU;
constexpr uint8_t kAccelPwrCtrlReg = 0x7DU;
constexpr uint8_t kAccelSoftResetReg = 0x7EU;
constexpr uint8_t kAccelInt1IoCtrlReg = 0x53U;
constexpr uint8_t kAccelIntMapDataReg = 0x58U;

// ===== 陀螺仪寄存器地址 =====
constexpr uint8_t kGyroChipIdReg = 0x00U;
constexpr uint8_t kGyroChipId = 0x0FU;
constexpr uint8_t kGyroRangeReg = 0x0FU;
constexpr uint8_t kGyroBandwidthReg = 0x10U;
constexpr uint8_t kGyroLpm1Reg = 0x11U;
constexpr uint8_t kGyroSoftResetReg = 0x14U;
constexpr uint8_t kGyroCtrlReg = 0x15U;
constexpr uint8_t kGyroInt3Int4IoConfReg = 0x16U;
constexpr uint8_t kGyroInt3Int4IoMapReg = 0x18U;

// ===== 配置值 =====
constexpr uint8_t kSoftResetCmd = 0xB6U;
constexpr uint8_t kAccelPwrConfActive = 0x00U;
constexpr uint8_t kAccelPwrCtrlOn = 0x04U;
constexpr uint8_t kAccelRange6G = 0x01U;
constexpr uint8_t kAccelBandwidthNormal = HwConfig::Bmi088::kAccelBandwidthRegValue;

// 加速度计 INT1 配置: 输出使能 + 推挽 + 低电平有效。
constexpr uint8_t kAccelInt1IoCtrl = 0x0CU;
// 加速度计中断映射: DRDY 映射到 INT1。
constexpr uint8_t kAccelIntMapDataDrdy = 0x04U;

constexpr uint8_t kGyroRange2000Dps = 0x00U;
constexpr uint8_t kGyroBandwidth1000Hz = HwConfig::Bmi088::kGyroBandwidthRegValue;
constexpr uint8_t kGyroNormalMode = 0x00U;
// 陀螺仪 DRDY 使能。
constexpr uint8_t kGyroDrdyOn = 0x80U;
// 陀螺仪 INT3 配置: 推挽 + 低电平有效。
constexpr uint8_t kGyroInt3Conf = 0x01U;
// 陀螺仪 DRDY 映射到 INT3。
constexpr uint8_t kGyroDrdyMapInt3 = 0x01U;

// 温度数据起始寄存器地址（TEMP_MSB）。
constexpr uint8_t kTempDataStartReg = 0x22U;

// BMI088 通信等待时间（us）。
constexpr uint16_t kComWaitSensorTimeUs = 150U;
// BMI088 长延时（ms），用于软复位等。
constexpr uint16_t kLongDelayMs = 80U;

// ===== 物理量换算常量 =====
constexpr float kDegToRad = 0.01745329252f;
// 加速度原始值到 m/s^2 的比例系数（±6g 量程）。//这里按照手册写的有问题 抄的战队库
// constexpr float kAccelScaleMps2 = (128.0f * 1000.0f * 1.5f) / 32768.0f;
constexpr float kAccelScaleMps2 = 0.00179443359375f;
// 陀螺仪原始值到 rad/s 的比例系数（±2000 dps 量程）。
constexpr float kGyroScaleRads = (2000.0f * kDegToRad) / 32768.0f;

} // namespace

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

bool Bmi088::Init()
{
	if ((hspi_ == nullptr) || (accel_cs_port_ == nullptr) || (gyro_cs_port_ == nullptr)) {
		return false;
	}

	DrvGPIO::SetPin(accel_cs_port_, accel_cs_pin_);
	DrvGPIO::SetPin(gyro_cs_port_, gyro_cs_pin_);

	// === 加速度计初始化 ===
	{
		// 通信检查：读取芯片 ID 两次。
		uint8_t id = 0U;
		ReadAccelSingleReg(kAccelChipIdReg, &id);
		Delay_us(kComWaitSensorTimeUs);
		ReadAccelSingleReg(kAccelChipIdReg, &id);
		Delay_us(kComWaitSensorTimeUs);

		// 软复位。
		WriteAccelReg(kAccelSoftResetReg, kSoftResetCmd);
		Delay_ms(kLongDelayMs);

		// 复位后通信检查。
		ReadAccelSingleReg(kAccelChipIdReg, &id);
		Delay_us(kComWaitSensorTimeUs);
		ReadAccelSingleReg(kAccelChipIdReg, &id);
		Delay_us(kComWaitSensorTimeUs);

		if (id != kAccelChipId) {
			return false;
		}

		// 配置寄存器并回读验证。
		struct AccelRegCfg {
			uint8_t reg;
			uint8_t val;
		};
		const AccelRegCfg accel_cfg[] = {
			{kAccelPwrCtrlReg,    kAccelPwrCtrlOn},
			{kAccelPwrConfReg,    kAccelPwrConfActive},
			{kAccelConfReg,       kAccelBandwidthNormal},
			{kAccelRangeReg,      kAccelRange6G},
			{kAccelInt1IoCtrlReg, kAccelInt1IoCtrl},
			{kAccelIntMapDataReg, kAccelIntMapDataDrdy},
		};

		for (const auto& cfg : accel_cfg) {
			WriteAccelReg(cfg.reg, cfg.val);
			Delay_us(kComWaitSensorTimeUs);

			uint8_t readback = 0U;
			ReadAccelSingleReg(cfg.reg, &readback);
			Delay_us(kComWaitSensorTimeUs);

			if (readback != cfg.val) {
				return false;
			}
		}
	}

	// === 陀螺仪初始化 ===
	{
		// 通信检查。
		uint8_t id = 0U;
		ReadGyroSingleReg(kGyroChipIdReg, &id);
		Delay_us(kComWaitSensorTimeUs);
		ReadGyroSingleReg(kGyroChipIdReg, &id);
		Delay_us(kComWaitSensorTimeUs);

		// 软复位。
		WriteGyroReg(kGyroSoftResetReg, kSoftResetCmd);
		Delay_ms(kLongDelayMs);

		// 复位后通信检查。
		ReadGyroSingleReg(kGyroChipIdReg, &id);
		Delay_us(kComWaitSensorTimeUs);
		ReadGyroSingleReg(kGyroChipIdReg, &id);
		Delay_us(kComWaitSensorTimeUs);

		if (id != kGyroChipId) {
			return false;
		}

		// 配置寄存器并回读验证。
		struct GyroRegCfg {
			uint8_t reg;
			uint8_t val;
		};
		const GyroRegCfg gyro_cfg[] = {
			{kGyroRangeReg,           kGyroRange2000Dps},
			{kGyroBandwidthReg,       kGyroBandwidth1000Hz},
			{kGyroLpm1Reg,            kGyroNormalMode},
			{kGyroCtrlReg,            kGyroDrdyOn},
			{kGyroInt3Int4IoConfReg,  kGyroInt3Conf},
			{kGyroInt3Int4IoMapReg,   kGyroDrdyMapInt3},
		};

		for (const auto& cfg : gyro_cfg) {
			WriteGyroReg(cfg.reg, cfg.val);
			Delay_us(kComWaitSensorTimeUs);

			uint8_t readback = 0U;
			ReadGyroSingleReg(cfg.reg, &readback);
			Delay_us(kComWaitSensorTimeUs);

			if (readback != cfg.val) {
				return false;
			}
		}
	}

	return true;
}

bool Bmi088::ReadSensor(Data* outData)
{
	if (outData == nullptr) {
		return false;
	}

	// 读取加速度 3 轴原始字节流（6 字节从 0x12 开始）。
	uint8_t accel_buf[6] = {0U};
	if (!ReadAccelMultiRegs(kAccelDataStartReg, accel_buf, 6U)) {
		return false;
	}

	// 读取陀螺仪 8 字节（从 0x00 开始，含芯片 ID + 保留字节 + 3 轴数据）。
	uint8_t gyro_buf[8] = {0U};
	if (!ReadGyroMultiRegs(kGyroChipIdReg, gyro_buf, 8U)) {
		return false;
	}

	// 字节流拼接为 16 位有符号原始值。
	outData->raw.accel_x = ToInt16(accel_buf[0], accel_buf[1]);
	outData->raw.accel_y = ToInt16(accel_buf[2], accel_buf[3]);
	outData->raw.accel_z = ToInt16(accel_buf[4], accel_buf[5]);

	// 验证陀螺仪芯片 ID，提取轴数据（buf[2..7]）。
	if (gyro_buf[0] == kGyroChipId) {
		outData->raw.gyro_x = ToInt16(gyro_buf[2], gyro_buf[3]);
		outData->raw.gyro_y = ToInt16(gyro_buf[4], gyro_buf[5]);
		outData->raw.gyro_z = ToInt16(gyro_buf[6], gyro_buf[7]);
	}

	// 原始计数值转换为物理单位。
	outData->accel_mps2[0] = static_cast<float>(outData->raw.accel_x) * kAccelScaleMps2;
	outData->accel_mps2[1] = static_cast<float>(outData->raw.accel_y) * kAccelScaleMps2;
	outData->accel_mps2[2] = static_cast<float>(outData->raw.accel_z) * kAccelScaleMps2;

	outData->gyro_rads[0] = static_cast<float>(outData->raw.gyro_x) * kGyroScaleRads;
	outData->gyro_rads[1] = static_cast<float>(outData->raw.gyro_y) * kGyroScaleRads;
	outData->gyro_rads[2] = static_cast<float>(outData->raw.gyro_z) * kGyroScaleRads;

	// 保存最近一次数据，供调试 Watch 窗口查看。
	last_data_ = *outData;

	// 低通滤波。
	if (!filter_inited_) {
		filtered_data_ = *outData;
		filter_inited_ = true;
	} else {
		const float a = filter_alpha_;
		const float b = 1.0f - a;
		for (int i = 0; i < 3; ++i) {
			filtered_data_.accel_mps2[i] = a * outData->accel_mps2[i] + b * filtered_data_.accel_mps2[i];
			filtered_data_.gyro_rads[i]  = a * outData->gyro_rads[i]  + b * filtered_data_.gyro_rads[i];
		}
	}

	return true;
}

bool Bmi088::WriteAccelReg(uint8_t regAddr, uint8_t data)
{
	return DrvSPI::WriteRegister(hspi_, accel_cs_port_, accel_cs_pin_, regAddr, data);
}

bool Bmi088::ReadAccelSingleReg(uint8_t regAddr, uint8_t* pData)
{
	return DrvSPI::AccelReadSingle(hspi_, accel_cs_port_, accel_cs_pin_, regAddr, pData);
}

bool Bmi088::ReadAccelMultiRegs(uint8_t regAddr, uint8_t* pData, uint16_t size)
{
	return DrvSPI::AccelReadMulti(hspi_, accel_cs_port_, accel_cs_pin_, regAddr, pData, size);
}

bool Bmi088::WriteGyroReg(uint8_t regAddr, uint8_t data)
{
	return DrvSPI::WriteRegister(hspi_, gyro_cs_port_, gyro_cs_pin_, regAddr, data);
}

bool Bmi088::ReadGyroSingleReg(uint8_t regAddr, uint8_t* pData)
{
	return DrvSPI::GyroReadSingle(hspi_, gyro_cs_port_, gyro_cs_pin_, regAddr, pData);
}

bool Bmi088::ReadGyroMultiRegs(uint8_t regAddr, uint8_t* pData, uint16_t size)
{
	return DrvSPI::GyroReadMulti(hspi_, gyro_cs_port_, gyro_cs_pin_, regAddr, pData, size);
}

bool Bmi088::ReadTemperature(float* outTempC)
{
	if (outTempC == nullptr) {
		return false;
	}

	uint8_t buf[2] = {0U};
	if (!ReadAccelMultiRegs(kTempDataStartReg, buf, 2U)) {
		return false;
	}

	// BMI088 温度格式: TEMP_MSB 为 bits[10:3]，TEMP_LSB 的 bits[7:5] 为 bits[2:0]。
	// 拼接为 11 位有符号值后换算: T = raw * 0.125 + 23 ℃。
	int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 3U) | (buf[1] >> 5U));
	if (raw > 1023) {
		raw -= 2048;
	}
	*outTempC = static_cast<float>(raw) * 0.125f + 23.0f;

	return true;
}

int16_t Bmi088::ToInt16(uint8_t lsb, uint8_t msb)
{
	return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8U) | static_cast<uint16_t>(lsb));
}
