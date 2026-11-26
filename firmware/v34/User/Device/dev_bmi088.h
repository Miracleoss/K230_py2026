#ifndef USER_DEVICE_DEV_BMI088_H
#define USER_DEVICE_DEV_BMI088_H

#include "Common/common_types.h"
#include "Common/imu_data.h"
#include "Driver/drv_gpio_pin.h"
#include "Driver/drv_spi_bus.h"

struct Bmi088RawSample {
    // 仅表示 BMI088 寄存器读出的传感器坐标系原始数据，供底层诊断和 Watch 使用；
    // 不得直接供串口协议、姿态算法或控制算法消费。
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature_raw;
    float temperature_deg_c;
};

struct Bmi088ProbeTrace {
    ResultCode acc_activation_result;
    ResultCode acc_id_result;
    ResultCode gyro_id_result;
    uint8_t acc_activation_rx[3];
    uint8_t acc_id_rx[3];
    uint8_t gyro_id_rx[2];
};

/**
 * 将一份已完成 SI 换算、仍处于 BMI088 传感器坐标系的 ImuData 转为机体坐标系。
 * 机体坐标系为 +X 前、+Y 上、+Z 右，轴映射为 body_x=-sensor_y、body_y=sensor_x、body_z=sensor_z。
 * 这是唯一的传感器到机体坐标变换边界，输入和输出均不修改外部状态。
 */
ImuData Bmi088SensorImuDataToBody(const ImuData& sensor_data);

/**
 * 将一份完整 BMI088 原始样本转换为机体坐标系 SI 数据（兼容仅提供毫秒时间戳的调用方）。
 * 原始 int16_t 值先转为 float/SI 单位，再由上面的纯坐标转换函数换轴和取负，
 * 避免直接对 INT16_MIN 做 int16_t 取负。
 */
ImuData Bmi088RawSampleToImuData(
    const Bmi088RawSample& sample,
    uint32_t sample_timestamp_ms,
    uint32_t data_ready_seq);

/**
 * 将一份完整 BMI088 原始样本转换为机体坐标系 SI 数据，并同时记录毫秒与硬件 1 us 微秒时间戳。
 */
ImuData Bmi088RawSampleToImuData(
    const Bmi088RawSample& sample,
    uint32_t sample_timestamp_ms,
    uint64_t sample_timestamp_us,
    uint32_t data_ready_seq);

typedef void (*Bmi088DelayMsFunction)(uint32_t delay_ms);

class Class_Bmi088 {
public:
    Class_Bmi088();

    ResultCode Bind(Class_SpiBus* spi_bus, Class_GpioPin* acc_cs, Class_GpioPin* gyro_cs);
    ResultCode Probe(uint8_t& acc_chip_id, uint8_t& gyro_chip_id);
    ResultCode ProbeWithTrace(uint8_t& acc_chip_id, uint8_t& gyro_chip_id, Bmi088ProbeTrace& trace);
    ResultCode Initialize(Bmi088DelayMsFunction delay_ms);
    ResultCode ReadRawSample(Bmi088RawSample& sample);
    bool IsBound() const;
    bool IsInitialized() const;

private:
    ResultCode ActivateAccSpi();
    ResultCode ReadAccRegister(uint8_t reg_addr, uint8_t& value);
    ResultCode ReadGyroRegister(uint8_t reg_addr, uint8_t& value);
    ResultCode WriteAccRegister(uint8_t reg_addr, uint8_t value);
    ResultCode WriteGyroRegister(uint8_t reg_addr, uint8_t value);
    ResultCode ReadAccRegisters(uint8_t reg_addr, uint8_t* values, uint8_t length);
    ResultCode ReadGyroRegisters(uint8_t reg_addr, uint8_t* values, uint8_t length);
    ResultCode VerifyAccRegister(uint8_t reg_addr, uint8_t expected_value);
    ResultCode VerifyGyroRegister(uint8_t reg_addr, uint8_t expected_value, uint8_t mask);
    ResultCode TransferWithChipSelect(
        Class_GpioPin* chip_select,
        const uint8_t* tx_buffer,
        uint8_t* rx_buffer,
        uint16_t length);
    static int16_t DecodeLittleEndian(const uint8_t* bytes);
    static int16_t DecodeTemperature(const uint8_t* bytes);

    Class_SpiBus* spi_bus_;
    Class_GpioPin* acc_cs_;
    Class_GpioPin* gyro_cs_;
    bool initialized_;
};

#endif /* USER_DEVICE_DEV_BMI088_H */
