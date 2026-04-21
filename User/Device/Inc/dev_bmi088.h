/**
 * @file dev_bmi088.h
 * @brief BMI088 惯导传感器设备层接口。
 * @details 封装 BMI088（加速度计+陀螺仪）初始化与数据读取能力。
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
 * @brief BMI088 设备类。
 */
class Bmi088 {
public:
	/**
	 * @brief BMI088 原始数据结构（未经量纲转换）。
	 */
	struct RawData {
		/** X 轴加速度原始值。 */
		int16_t accel_x;
		/** Y 轴加速度原始值。 */
		int16_t accel_y;
		/** Z 轴加速度原始值。 */
		int16_t accel_z;
		/** X 轴角速度原始值。 */
		int16_t gyro_x;
		/** Y 轴角速度原始值。 */
		int16_t gyro_y;
		/** Z 轴角速度原始值。 */
		int16_t gyro_z;
	};

	/**
	 * @brief BMI088 输出数据结构（包含原始值与物理量）。
	 */
	struct Data {
		/** 传感器原始数据。 */
		RawData raw;
		/** 加速度物理量，单位 m/s^2，索引 0/1/2 对应 X/Y/Z。 */
		float accel_mps2[3];
		/** 角速度物理量，单位 rad/s，索引 0/1/2 对应 X/Y/Z。 */
		float gyro_rads[3];
	};

	/**
	 * @brief 构造函数。
	 * @param hspi SPI 句柄。
	 * @param accelCsPort 加速度计片选端口。
	 * @param accelCsPin 加速度计片选引脚。
	 * @param gyroCsPort 陀螺仪片选端口。
	 * @param gyroCsPin 陀螺仪片选引脚。
	 */
	Bmi088(SPI_HandleTypeDef* hspi,
		   GPIO_TypeDef* accelCsPort = SPI_CS0_GPIO_Port,
		   uint16_t accelCsPin = SPI_CS0_Pin,
		   GPIO_TypeDef* gyroCsPort = SPI_CS1_GPIO_Port,
		   uint16_t gyroCsPin = SPI_CS1_Pin);

	/**
	 * @brief 初始化 BMI088 加速度计与陀螺仪。
	 * @return true：初始化成功；false：通信失败或配置失败。
	 */
	bool Init();

	/**
	 * @brief 读取传感器数据并转换为物理量。
	 * @param outData 输出结构体指针。
	 * @return true：读取成功；false：读取失败。
	 */
	bool ReadSensor(Data* outData);

private:
	/** 写加速度计寄存器。 */
	bool WriteAccelReg(uint8_t regAddr, uint8_t data);
	/** 读加速度计连续寄存器。 */
	bool ReadAccelRegs(uint8_t regAddr, uint8_t* pData, uint16_t size);
	/** 写陀螺仪寄存器。 */
	bool WriteGyroReg(uint8_t regAddr, uint8_t data);
	/** 读陀螺仪连续寄存器。 */
	bool ReadGyroRegs(uint8_t regAddr, uint8_t* pData, uint16_t size);

	/** 将 2 个字节拼接为 int16_t。 */
	static int16_t ToInt16(uint8_t lsb, uint8_t msb);

	/** SPI 句柄。 */
	SPI_HandleTypeDef* hspi_;
	/** 加速度计片选端口。 */
	GPIO_TypeDef* accel_cs_port_;
	/** 加速度计片选引脚。 */
	uint16_t accel_cs_pin_;
	/** 陀螺仪片选端口。 */
	GPIO_TypeDef* gyro_cs_port_;
	/** 陀螺仪片选引脚。 */
	uint16_t gyro_cs_pin_;
};

#endif
