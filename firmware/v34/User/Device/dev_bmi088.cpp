#include "Device/dev_bmi088.h"

static const uint8_t kBmi088ReadMask = 0x80U;
static const uint8_t kBmi088AccChipIdRegister = 0x00U;
static const uint8_t kBmi088AccExpectedChipId = 0x1EU;
static const uint8_t kBmi088GyroChipIdRegister = 0x00U;
static const uint8_t kBmi088GyroExpectedChipId = 0x0FU;
static const uint32_t kBmi088SpiTimeoutMs = 10U;
static const uint8_t kBmi088AccSoftResetRegister = 0x7EU;
static const uint8_t kBmi088AccPowerConfigRegister = 0x7CU;
static const uint8_t kBmi088AccPowerControlRegister = 0x7DU;
static const uint8_t kBmi088AccConfigRegister = 0x40U;
static const uint8_t kBmi088AccRangeRegister = 0x41U;
static const uint8_t kBmi088AccDataRegister = 0x12U;
static const uint8_t kBmi088TemperatureRegister = 0x22U;
static const uint8_t kBmi088GyroSoftResetRegister = 0x14U;
static const uint8_t kBmi088GyroBandwidthRegister = 0x10U;
static const uint8_t kBmi088GyroRangeRegister = 0x0FU;
static const uint8_t kBmi088GyroDataRegister = 0x02U;
static const uint8_t kBmi088SoftResetValue = 0xB6U;
static const uint8_t kBmi088GyroBandwidthMask = 0x07U;
static const uint8_t kBmi088MaximumBurstLength = 6U;
static const float kBmi088AccCountsPerG = 5460.0f;
static const float kBmi088GyroCountsPerDegreePerSecond = 16.4f;
static const float kStandardGravityMps2 = 9.80665f;
static const float kPi = 3.14159265358979323846f;
static const float kDegreesPerRevolution = 180.0f;

ImuData Bmi088SensorImuDataToBody(const ImuData& sensor_data)
{
    ImuData body_data = sensor_data;
    // 只在这里执行一次 BMI088 传感器坐标系到机体坐标系（+X 前、+Y 上、+Z 右）的映射：
    // body_x = -sensor_y，body_y = sensor_x，body_z = sensor_z。
    body_data.specific_force_x_mps2 = -sensor_data.specific_force_y_mps2;
    body_data.specific_force_y_mps2 = sensor_data.specific_force_z_mps2;
    body_data.specific_force_z_mps2 = -sensor_data.specific_force_x_mps2;
    body_data.gyro_x_radps = -sensor_data.gyro_y_radps;
    body_data.gyro_y_radps = sensor_data.gyro_z_radps;
    body_data.gyro_z_radps = -sensor_data.gyro_x_radps;
    return body_data;
}

ImuData Bmi088RawSampleToImuData(
    const Bmi088RawSample& sample,
    uint32_t sample_timestamp_ms,
    uint32_t data_ready_seq)
{
    return Bmi088RawSampleToImuData(
        sample,
        sample_timestamp_ms,
        static_cast<uint64_t>(sample_timestamp_ms) * 1000ULL,
        data_ready_seq);
}

ImuData Bmi088RawSampleToImuData(
    const Bmi088RawSample& sample,
    uint32_t sample_timestamp_ms,
    uint64_t sample_timestamp_us,
    uint32_t data_ready_seq)
{
    // 此临时对象仍是传感器坐标系；它只存在于原始数据到规范化数据的边界内。
    ImuData sensor_data;
    const float acc_scale_mps2_per_count = kStandardGravityMps2 / kBmi088AccCountsPerG;
    const float gyro_scale_radps_per_count =
        kPi / (kBmi088GyroCountsPerDegreePerSecond * kDegreesPerRevolution);

    sensor_data.specific_force_x_mps2 = static_cast<float>(sample.acc_x) * acc_scale_mps2_per_count;
    sensor_data.specific_force_y_mps2 = static_cast<float>(sample.acc_y) * acc_scale_mps2_per_count;
    sensor_data.specific_force_z_mps2 = static_cast<float>(sample.acc_z) * acc_scale_mps2_per_count;
    sensor_data.gyro_x_radps = static_cast<float>(sample.gyro_x) * gyro_scale_radps_per_count;
    sensor_data.gyro_y_radps = static_cast<float>(sample.gyro_y) * gyro_scale_radps_per_count;
    sensor_data.gyro_z_radps = static_cast<float>(sample.gyro_z) * gyro_scale_radps_per_count;
    sensor_data.temperature_deg_c = sample.temperature_deg_c;
    sensor_data.sample_timestamp_ms = sample_timestamp_ms;
    sensor_data.sample_timestamp_us = sample_timestamp_us;
    sensor_data.data_ready_seq = data_ready_seq;
    sensor_data.valid = true;
    return Bmi088SensorImuDataToBody(sensor_data);
}

Class_Bmi088::Class_Bmi088()
    : spi_bus_(nullptr),
      acc_cs_(nullptr),
      gyro_cs_(nullptr),
      initialized_(false)
{
}

ResultCode Class_Bmi088::Bind(Class_SpiBus* spi_bus, Class_GpioPin* acc_cs, Class_GpioPin* gyro_cs)
{
    initialized_ = false;
    if (spi_bus == nullptr || acc_cs == nullptr || gyro_cs == nullptr) {
        return ResultCode::ConfigError;
    }

    spi_bus_ = spi_bus;
    acc_cs_ = acc_cs;
    gyro_cs_ = gyro_cs;
    return ResultCode::Ok;
}

bool Class_Bmi088::IsBound() const
{
    return spi_bus_ != nullptr && acc_cs_ != nullptr && gyro_cs_ != nullptr;
}

bool Class_Bmi088::IsInitialized() const
{
    return initialized_;
}

ResultCode Class_Bmi088::Probe(uint8_t& acc_chip_id, uint8_t& gyro_chip_id)
{
    Bmi088ProbeTrace ignored_trace = {};
    return ProbeWithTrace(acc_chip_id, gyro_chip_id, ignored_trace);
}

ResultCode Class_Bmi088::ProbeWithTrace(
    uint8_t& acc_chip_id,
    uint8_t& gyro_chip_id,
    Bmi088ProbeTrace& trace)
{
    Bmi088ProbeTrace candidate = {};
    candidate.acc_activation_result = ResultCode::NotReady;
    candidate.acc_id_result = ResultCode::NotReady;
    candidate.gyro_id_result = ResultCode::NotReady;
    if (!IsBound()) {
        trace = candidate;
        return ResultCode::NotReady;
    }

    const uint8_t acc_tx[3] = {
        static_cast<uint8_t>(kBmi088AccChipIdRegister | kBmi088ReadMask),
        0U,
        0U
    };
    candidate.acc_activation_result =
        TransferWithChipSelect(acc_cs_, acc_tx, candidate.acc_activation_rx, 3U);
    if (candidate.acc_activation_result != ResultCode::Ok) {
        trace = candidate;
        return candidate.acc_activation_result;
    }

    candidate.acc_id_result = TransferWithChipSelect(acc_cs_, acc_tx, candidate.acc_id_rx, 3U);
    if (candidate.acc_id_result != ResultCode::Ok) {
        trace = candidate;
        return candidate.acc_id_result;
    }
    acc_chip_id = candidate.acc_id_rx[2];

    const uint8_t gyro_tx[2] = {
        static_cast<uint8_t>(kBmi088GyroChipIdRegister | kBmi088ReadMask),
        0U
    };
    candidate.gyro_id_result = TransferWithChipSelect(gyro_cs_, gyro_tx, candidate.gyro_id_rx, 2U);
    if (candidate.gyro_id_result != ResultCode::Ok) {
        trace = candidate;
        return candidate.gyro_id_result;
    }
    gyro_chip_id = candidate.gyro_id_rx[1];
    trace = candidate;

    if (acc_chip_id != kBmi088AccExpectedChipId || gyro_chip_id != kBmi088GyroExpectedChipId) {
        return ResultCode::InvalidData;
    }
    return ResultCode::Ok;
}

ResultCode Class_Bmi088::Initialize(Bmi088DelayMsFunction delay_ms)
{
    initialized_ = false;
    if (!IsBound()) {
        return ResultCode::NotReady;
    }
    if (delay_ms == nullptr) {
        return ResultCode::ConfigError;
    }

    ResultCode result = acc_cs_->SetHigh();
    if (result != ResultCode::Ok) {
        return result;
    }
    result = gyro_cs_->SetHigh();
    if (result != ResultCode::Ok) {
        return result;
    }

    // ACC always powers up in I2C mode. A completed dummy SPI read supplies
    // the CSB1 rising edge that selects SPI before the first register write.
    result = ActivateAccSpi();
    if (result != ResultCode::Ok) {
        return result;
    }
    result = WriteAccRegister(kBmi088AccSoftResetRegister, kBmi088SoftResetValue);
    if (result != ResultCode::Ok) {
        return result;
    }
    delay_ms(1U);
    result = ActivateAccSpi();
    if (result != ResultCode::Ok) {
        return result;
    }
    result = WriteGyroRegister(kBmi088GyroSoftResetRegister, kBmi088SoftResetValue);
    if (result != ResultCode::Ok) {
        return result;
    }
    delay_ms(30U);

    uint8_t acc_chip_id = 0U;
    uint8_t gyro_chip_id = 0U;
    result = Probe(acc_chip_id, gyro_chip_id);
    if (result != ResultCode::Ok) {
        return result;
    }

    result = WriteAccRegister(kBmi088AccPowerConfigRegister, 0x00U);
    if (result != ResultCode::Ok) {
        return result;
    }
    delay_ms(5U);
    result = WriteAccRegister(kBmi088AccPowerControlRegister, 0x04U);
    if (result != ResultCode::Ok) {
        return result;
    }
    delay_ms(5U);
    // ACC_CONF (0x40): ODR = 1600 Hz, OSR4 four-fold oversampling (145 Hz anti-aliasing BW)
    result = WriteAccRegister(kBmi088AccConfigRegister, 0x8CU);
    if (result != ResultCode::Ok) {
        return result;
    }
    delay_ms(40U);
    result = WriteAccRegister(kBmi088AccRangeRegister, 0x01U);
    if (result != ResultCode::Ok) {
        return result;
    }
    delay_ms(1U);
    // GYRO_BANDWIDTH (0x10): ODR = 1000 Hz, 116 Hz hardware low-pass BW
    result = WriteGyroRegister(kBmi088GyroBandwidthRegister, 0x02U);
    if (result != ResultCode::Ok) {
        return result;
    }
    delay_ms(1U);
    result = WriteGyroRegister(kBmi088GyroRangeRegister, 0x00U);
    if (result != ResultCode::Ok) {
        return result;
    }
    delay_ms(10U);

    result = VerifyAccRegister(kBmi088AccPowerConfigRegister, 0x00U);
    if (result != ResultCode::Ok) {
        return result;
    }
    result = VerifyAccRegister(kBmi088AccPowerControlRegister, 0x04U);
    if (result != ResultCode::Ok) {
        return result;
    }
    result = VerifyAccRegister(kBmi088AccConfigRegister, 0x8CU);
    if (result != ResultCode::Ok) {
        return result;
    }
    result = VerifyAccRegister(kBmi088AccRangeRegister, 0x01U);
    if (result != ResultCode::Ok) {
        return result;
    }
    result = VerifyGyroRegister(kBmi088GyroBandwidthRegister, 0x02U, kBmi088GyroBandwidthMask);
    if (result != ResultCode::Ok) {
        return result;
    }
    result = VerifyGyroRegister(kBmi088GyroRangeRegister, 0x00U, 0xFFU);
    if (result != ResultCode::Ok) {
        return result;
    }

    initialized_ = true;
    return ResultCode::Ok;
}

ResultCode Class_Bmi088::ReadRawSample(Bmi088RawSample& sample)
{
    if (!initialized_) {
        return ResultCode::NotReady;
    }

    uint8_t acc_values[6] = {0U};
    uint8_t gyro_values[6] = {0U};
    uint8_t temperature_values[2] = {0U};
    ResultCode result = ReadAccRegisters(kBmi088AccDataRegister, acc_values, 6U);
    if (result != ResultCode::Ok) {
        return result;
    }
    result = ReadGyroRegisters(kBmi088GyroDataRegister, gyro_values, 6U);
    if (result != ResultCode::Ok) {
        return result;
    }
    result = ReadAccRegisters(kBmi088TemperatureRegister, temperature_values, 2U);
    if (result != ResultCode::Ok) {
        return result;
    }

    Bmi088RawSample candidate = {};
    candidate.acc_x = DecodeLittleEndian(&acc_values[0]);
    candidate.acc_y = DecodeLittleEndian(&acc_values[2]);
    candidate.acc_z = DecodeLittleEndian(&acc_values[4]);
    candidate.gyro_x = DecodeLittleEndian(&gyro_values[0]);
    candidate.gyro_y = DecodeLittleEndian(&gyro_values[2]);
    candidate.gyro_z = DecodeLittleEndian(&gyro_values[4]);
    candidate.temperature_raw = DecodeTemperature(temperature_values);
    candidate.temperature_deg_c = static_cast<float>(candidate.temperature_raw) * 0.125f + 23.0f;
    sample = candidate;
    return ResultCode::Ok;
}

ResultCode Class_Bmi088::ActivateAccSpi()
{
    uint8_t ignored_value = 0U;
    return ReadAccRegister(kBmi088AccChipIdRegister, ignored_value);
}

ResultCode Class_Bmi088::ReadAccRegister(uint8_t reg_addr, uint8_t& value)
{
    return ReadAccRegisters(reg_addr, &value, 1U);
}

ResultCode Class_Bmi088::ReadGyroRegister(uint8_t reg_addr, uint8_t& value)
{
    return ReadGyroRegisters(reg_addr, &value, 1U);
}

ResultCode Class_Bmi088::WriteAccRegister(uint8_t reg_addr, uint8_t value)
{
    const uint8_t tx_buffer[2] = {reg_addr, value};
    uint8_t rx_buffer[2] = {0U};
    return TransferWithChipSelect(acc_cs_, tx_buffer, rx_buffer, 2U);
}

ResultCode Class_Bmi088::WriteGyroRegister(uint8_t reg_addr, uint8_t value)
{
    const uint8_t tx_buffer[2] = {reg_addr, value};
    uint8_t rx_buffer[2] = {0U};
    return TransferWithChipSelect(gyro_cs_, tx_buffer, rx_buffer, 2U);
}

ResultCode Class_Bmi088::ReadAccRegisters(uint8_t reg_addr, uint8_t* values, uint8_t length)
{
    if (values == nullptr || length == 0U || length > kBmi088MaximumBurstLength) {
        return ResultCode::InvalidData;
    }

    uint8_t tx_buffer[8] = {0U};
    uint8_t rx_buffer[8] = {0U};
    tx_buffer[0] = static_cast<uint8_t>(reg_addr | kBmi088ReadMask);
    const ResultCode result = TransferWithChipSelect(acc_cs_, tx_buffer, rx_buffer, static_cast<uint16_t>(length + 2U));
    if (result != ResultCode::Ok) {
        return result;
    }
    for (uint8_t index = 0U; index < length; ++index) {
        values[index] = rx_buffer[index + 2U];
    }
    return ResultCode::Ok;
}

ResultCode Class_Bmi088::ReadGyroRegisters(uint8_t reg_addr, uint8_t* values, uint8_t length)
{
    if (values == nullptr || length == 0U || length > kBmi088MaximumBurstLength) {
        return ResultCode::InvalidData;
    }

    uint8_t tx_buffer[7] = {0U};
    uint8_t rx_buffer[7] = {0U};
    tx_buffer[0] = static_cast<uint8_t>(reg_addr | kBmi088ReadMask);
    const ResultCode result = TransferWithChipSelect(gyro_cs_, tx_buffer, rx_buffer, static_cast<uint16_t>(length + 1U));
    if (result != ResultCode::Ok) {
        return result;
    }
    for (uint8_t index = 0U; index < length; ++index) {
        values[index] = rx_buffer[index + 1U];
    }
    return ResultCode::Ok;
}

ResultCode Class_Bmi088::VerifyAccRegister(uint8_t reg_addr, uint8_t expected_value)
{
    uint8_t value = 0U;
    const ResultCode result = ReadAccRegister(reg_addr, value);
    if (result != ResultCode::Ok) {
        return result;
    }
    return value == expected_value ? ResultCode::Ok : ResultCode::InvalidData;
}

ResultCode Class_Bmi088::VerifyGyroRegister(uint8_t reg_addr, uint8_t expected_value, uint8_t mask)
{
    uint8_t value = 0U;
    const ResultCode result = ReadGyroRegister(reg_addr, value);
    if (result != ResultCode::Ok) {
        return result;
    }
    return (value & mask) == (expected_value & mask) ? ResultCode::Ok : ResultCode::InvalidData;
}

ResultCode Class_Bmi088::TransferWithChipSelect(
    Class_GpioPin* chip_select,
    const uint8_t* tx_buffer,
    uint8_t* rx_buffer,
    uint16_t length)
{
    if (!IsBound()) {
        return ResultCode::NotReady;
    }

    ResultCode result = chip_select->SetLow();
    if (result != ResultCode::Ok) {
        return result;
    }
    result = spi_bus_->Transfer(tx_buffer, rx_buffer, length, kBmi088SpiTimeoutMs);
    const ResultCode high_result = chip_select->SetHigh();
    if (result != ResultCode::Ok) {
        return result;
    }
    return high_result;
}

int16_t Class_Bmi088::DecodeLittleEndian(const uint8_t* bytes)
{
    const uint16_t combined = static_cast<uint16_t>(bytes[0]) |
                              static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8U);
    return static_cast<int16_t>(combined);
}

int16_t Class_Bmi088::DecodeTemperature(const uint8_t* bytes)
{
    int16_t raw_value = static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) << 3U) |
                        static_cast<int16_t>(bytes[1] >> 5U);
    if (raw_value > 1023) {
        raw_value = static_cast<int16_t>(raw_value - 2048);
    }
    return raw_value;
}
