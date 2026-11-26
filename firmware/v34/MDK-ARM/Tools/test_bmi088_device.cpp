#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "Device/dev_bmi088.h"
#include "Common/imu_data.h"

enum class EventType : uint8_t {
    AccWrite,
    GyroWrite,
    AccSpiReactivation,
    AccRead,
    GyroRead,
    Delay
};

struct RecordedEvent {
    EventType type;
    uint8_t address;
    uint8_t value;
};

static uint8_t g_acc_registers[256];
static uint8_t g_gyro_registers[256];
static RecordedEvent g_events[48];
static uint32_t g_event_count;
static uint32_t g_transfer_count;
static uint32_t g_fail_transfer_number;
static uint16_t g_transfer_lengths[64];
static uint32_t g_transfer_length_count;
static bool g_acc_cs_low;
static bool g_gyro_cs_low;
static bool g_acc_spi_active;
static uint32_t g_elapsed_ms;
static uint32_t g_gyro_quiet_until_ms;
static bool g_corrupt_readback;
static bool g_corrupt_accelerometer;
static uint8_t g_corrupt_address;
static uint8_t g_corrupt_mask;
static uint8_t g_last_gyro_bandwidth_read_value;

static GPIO_TypeDef g_acc_port;
static GPIO_TypeDef g_gyro_port;
static SPI_HandleTypeDef g_spi_handle;
static const uint16_t kAccPin = 1U;
static const uint16_t kGyroPin = 2U;

static void RecordEvent(EventType type, uint8_t address, uint8_t value)
{
    if (g_event_count < 48U) {
        g_events[g_event_count].type = type;
        g_events[g_event_count].address = address;
        g_events[g_event_count].value = value;
        ++g_event_count;
    }
}

extern "C" void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state)
{
    if (port == &g_acc_port && pin == kAccPin) {
        g_acc_cs_low = (state == GPIO_PIN_RESET);
    }
    if (port == &g_gyro_port && pin == kGyroPin) {
        g_gyro_cs_low = (state == GPIO_PIN_RESET);
    }
}

extern "C" GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t)
{
    return GPIO_PIN_RESET;
}

extern "C" HAL_StatusTypeDef HAL_SPI_TransmitReceive(
    SPI_HandleTypeDef*, uint8_t* tx, uint8_t* rx, uint16_t length, uint32_t)
{
    ++g_transfer_count;
    g_transfer_lengths[g_transfer_length_count++] = length;
    if (g_fail_transfer_number != 0U && g_transfer_count == g_fail_transfer_number) {
        return HAL_ERROR;
    }

    const bool accelerometer = g_acc_cs_low;
    if (accelerometer == g_gyro_cs_low || length < 2U) {
        return HAL_ERROR;
    }

    const uint8_t address = static_cast<uint8_t>(tx[0] & 0x7FU);
    if ((tx[0] & 0x80U) == 0U) {
        if (length != 2U || (!accelerometer &&
                              address != 0x14U && g_elapsed_ms < g_gyro_quiet_until_ms) ||
            (accelerometer && !g_acc_spi_active)) {
            return HAL_ERROR;
        }
        uint8_t* const registers = accelerometer ? g_acc_registers : g_gyro_registers;
        registers[address] = tx[1];
        RecordEvent(accelerometer ? EventType::AccWrite : EventType::GyroWrite, address, tx[1]);
        if (accelerometer && address == 0x7EU && tx[1] == 0xB6U) {
            g_acc_spi_active = false;
        }
        if (!accelerometer && address == 0x14U && tx[1] == 0xB6U) {
            g_gyro_quiet_until_ms = g_elapsed_ms + 30U;
        }
        return HAL_OK;
    }

    if (!accelerometer && g_elapsed_ms < g_gyro_quiet_until_ms) {
        return HAL_ERROR;
    }
    if (accelerometer && !g_acc_spi_active) {
        memset(rx, 0, length);
        g_acc_spi_active = true;
        RecordEvent(EventType::AccSpiReactivation, address, 0U);
        return HAL_OK;
    }
    const uint16_t payload_offset = accelerometer ? 2U : 1U;
    if (length <= payload_offset) {
        return HAL_ERROR;
    }
    uint8_t* const registers = accelerometer ? g_acc_registers : g_gyro_registers;
    memset(rx, 0, length);
    for (uint16_t index = payload_offset; index < length; ++index) {
        uint8_t value = registers[static_cast<uint8_t>(address + index - payload_offset)];
        if (!accelerometer && address == 0x10U) {
            value |= 0x80U;
        }
        if (g_corrupt_readback && accelerometer == g_corrupt_accelerometer &&
            address == g_corrupt_address) {
            value ^= g_corrupt_mask;
        }
        rx[index] = value;
    }
    const uint8_t first_value = rx[payload_offset];
    RecordEvent(accelerometer ? EventType::AccRead : EventType::GyroRead, address, first_value);
    if (!accelerometer && address == 0x10U) {
        g_last_gyro_bandwidth_read_value = first_value;
    }
    return HAL_OK;
}

static void RecordDelayMs(uint32_t delay_ms)
{
    RecordEvent(EventType::Delay, 0U, static_cast<uint8_t>(delay_ms));
    g_elapsed_ms += delay_ms;
}

static bool Expect(bool condition, const char* message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        return false;
    }
    return true;
}

static bool ExpectNear(float actual, float expected, float tolerance, const char* message)
{
    return Expect(fabsf(actual - expected) <= tolerance, message);
}

static bool ExpectBmi088RawToImuConversion()
{
    const float kStandardGravityMps2 = 9.80665f;
    const float kPi = 3.14159265358979323846f;
    const float kExpectedTenDegPerSecondRadps = 10.0f * kPi / 180.0f;
    const float kExpectedFullScaleGyroRadps = 32767.0f / 16.4f * kPi / 180.0f;

    const Bmi088RawSample raw = {
        5460,
        -5460,
        0,
        164,
        -164,
        0,
        0,
        23.5f
    };
    const ImuData converted = Bmi088RawSampleToImuData(raw, 1234U, 7U);

    bool passed = true;
    passed &= ExpectNear(converted.specific_force_x_mps2, kStandardGravityMps2, 0.0001f,
                         "body ACC X must equal negative sensor ACC Y after SI conversion");
    passed &= ExpectNear(converted.specific_force_y_mps2, kStandardGravityMps2, 0.0001f,
                         "body ACC Y must equal sensor ACC X after SI conversion");
    passed &= ExpectNear(converted.specific_force_z_mps2, 0.0f, 0.0001f,
                         "body ACC Z must equal sensor ACC Z after SI conversion");
    passed &= ExpectNear(converted.gyro_x_radps, kExpectedTenDegPerSecondRadps, 0.0001f,
                         "body GYRO X must equal negative sensor GYRO Y after SI conversion");
    passed &= ExpectNear(converted.gyro_y_radps, kExpectedTenDegPerSecondRadps, 0.0001f,
                         "body GYRO Y must equal sensor GYRO X after SI conversion");
    passed &= ExpectNear(converted.gyro_z_radps, 0.0f, 0.0001f,
                         "body GYRO Z must equal sensor GYRO Z after SI conversion");
    passed &= Expect(converted.temperature_deg_c == 23.5f,
                     "raw conversion must preserve sampled temperature");
    passed &= Expect(converted.sample_timestamp_ms == 1234U && converted.data_ready_seq == 7U,
                     "raw conversion must preserve sample timing metadata");
    passed &= Expect(converted.valid, "a converted successful sample must be valid");

    const Bmi088RawSample full_scale = {
        32760,
        -32760,
        32760,
        32767,
        -32767,
        32767,
        0,
        0.0f
    };
    const ImuData full_scale_converted = Bmi088RawSampleToImuData(full_scale, 0U, 0U);
    passed &= ExpectNear(full_scale_converted.specific_force_x_mps2,
                         6.0f * kStandardGravityMps2, 0.001f,
                         "body ACC X must use negative sensor ACC Y at the +/-6 g sensitivity");
    passed &= ExpectNear(full_scale_converted.specific_force_y_mps2,
                         6.0f * kStandardGravityMps2, 0.001f,
                         "body ACC Y must use sensor ACC X at the +/-6 g sensitivity");
    passed &= ExpectNear(full_scale_converted.specific_force_z_mps2,
                         6.0f * kStandardGravityMps2, 0.001f,
                         "body ACC Z must use sensor ACC Z at the +/-6 g sensitivity");
    passed &= ExpectNear(full_scale_converted.gyro_x_radps,
                         kExpectedFullScaleGyroRadps, 0.01f,
                         "body GYRO X must use negative sensor GYRO Y at the +/-2000 deg/s sensitivity");
    passed &= ExpectNear(full_scale_converted.gyro_y_radps,
                         kExpectedFullScaleGyroRadps, 0.01f,
                         "body GYRO Y must use sensor GYRO X at the +/-2000 deg/s sensitivity");
    passed &= ExpectNear(full_scale_converted.gyro_z_radps,
                         kExpectedFullScaleGyroRadps, 0.01f,
                         "body GYRO Z must use sensor GYRO Z at the +/-2000 deg/s sensitivity");

    const Bmi088RawSample asymmetric_sensor = {
        1000,
        -2000,
        3000,
        -4000,
        5000,
        -6000,
        0,
        0.0f
    };
    const ImuData asymmetric_body =
        Bmi088RawSampleToImuData(asymmetric_sensor, 77U, 8U);
    const float acc_scale_mps2_per_count = kStandardGravityMps2 / 5460.0f;
    const float gyro_scale_radps_per_count = kPi / (16.4f * 180.0f);
    passed &= ExpectNear(asymmetric_body.specific_force_x_mps2,
                         2000.0f * acc_scale_mps2_per_count, 0.0001f,
                         "ACC sensor (1000, -2000, 3000) must map body X from negative sensor Y");
    passed &= ExpectNear(asymmetric_body.specific_force_y_mps2,
                         1000.0f * acc_scale_mps2_per_count, 0.0001f,
                         "ACC sensor (1000, -2000, 3000) must map body Y from sensor X");
    passed &= ExpectNear(asymmetric_body.specific_force_z_mps2,
                         3000.0f * acc_scale_mps2_per_count, 0.0001f,
                         "ACC sensor (1000, -2000, 3000) must map body Z from sensor Z");
    passed &= ExpectNear(asymmetric_body.gyro_x_radps,
                         -5000.0f * gyro_scale_radps_per_count, 0.0001f,
                         "GYRO sensor (-4000, 5000, -6000) must map body X from negative sensor Y");
    passed &= ExpectNear(asymmetric_body.gyro_y_radps,
                         -4000.0f * gyro_scale_radps_per_count, 0.0001f,
                         "GYRO sensor (-4000, 5000, -6000) must map body Y from sensor X");
    passed &= ExpectNear(asymmetric_body.gyro_z_radps,
                         -6000.0f * gyro_scale_radps_per_count, 0.0001f,
                         "GYRO sensor (-4000, 5000, -6000) must map body Z from sensor Z");

    const Bmi088RawSample minimum_sensor_axis = {
        static_cast<int16_t>(-32768),
        static_cast<int16_t>(-32768),
        static_cast<int16_t>(-32768),
        static_cast<int16_t>(-32768),
        static_cast<int16_t>(-32768),
        static_cast<int16_t>(-32768),
        0,
        0.0f
    };
    const ImuData minimum_sensor_axis_converted =
        Bmi088RawSampleToImuData(minimum_sensor_axis, 88U, 9U);
    passed &= ExpectNear(minimum_sensor_axis_converted.specific_force_x_mps2,
                         32768.0f * acc_scale_mps2_per_count, 0.0001f,
                         "sensor ACC Y INT16_MIN must negate in float without int16 overflow");
    passed &= ExpectNear(minimum_sensor_axis_converted.specific_force_y_mps2,
                         -32768.0f * acc_scale_mps2_per_count, 0.0001f,
                         "sensor ACC X INT16_MIN must remain negative in float without overflow");
    passed &= ExpectNear(minimum_sensor_axis_converted.specific_force_z_mps2,
                         -32768.0f * acc_scale_mps2_per_count, 0.0001f,
                         "sensor ACC Z INT16_MIN must remain negative in float without overflow");
    passed &= ExpectNear(minimum_sensor_axis_converted.gyro_x_radps,
                         32768.0f * gyro_scale_radps_per_count, 0.0001f,
                         "sensor GYRO Y INT16_MIN must negate in float without int16 overflow");
    passed &= ExpectNear(minimum_sensor_axis_converted.gyro_y_radps,
                         -32768.0f * gyro_scale_radps_per_count, 0.0001f,
                         "sensor GYRO X INT16_MIN must remain negative in float without overflow");
    passed &= ExpectNear(minimum_sensor_axis_converted.gyro_z_radps,
                         -32768.0f * gyro_scale_radps_per_count, 0.0001f,
                         "sensor GYRO Z INT16_MIN must remain negative in float without overflow");
    passed &= Expect(minimum_sensor_axis_converted.sample_timestamp_ms == 88U &&
                     minimum_sensor_axis_converted.data_ready_seq == 9U &&
                     minimum_sensor_axis_converted.temperature_deg_c == 0.0f &&
                     minimum_sensor_axis_converted.valid,
                     "coordinate conversion must preserve metadata and valid state");
    return passed;
}

static bool ExpectSensorToBodyMapping()
{
    ImuData sensor_data;
    sensor_data.specific_force_x_mps2 = 1.0f;
    sensor_data.specific_force_y_mps2 = 2.0f;
    sensor_data.specific_force_z_mps2 = 3.0f;
    sensor_data.gyro_x_radps = 4.0f;
    sensor_data.gyro_y_radps = 5.0f;
    sensor_data.gyro_z_radps = 6.0f;
    sensor_data.temperature_deg_c = 27.5f;
    sensor_data.sample_timestamp_ms = 123U;
    sensor_data.data_ready_seq = 456U;
    sensor_data.valid = false;

    const ImuData body_data = Bmi088SensorImuDataToBody(sensor_data);
    bool passed = true;
    passed &= Expect(body_data.specific_force_x_mps2 == -2.0f &&
                     body_data.specific_force_y_mps2 == 1.0f &&
                     body_data.specific_force_z_mps2 == 3.0f,
                     "sensor ACC (x, y, z) must map to body ACC (-y, x, z)");
    passed &= Expect(body_data.gyro_x_radps == -5.0f &&
                     body_data.gyro_y_radps == 4.0f &&
                     body_data.gyro_z_radps == 6.0f,
                     "sensor GYRO (x, y, z) must map to body GYRO (-y, x, z)");
    passed &= Expect(body_data.temperature_deg_c == 27.5f &&
                     body_data.sample_timestamp_ms == 123U &&
                     body_data.data_ready_seq == 456U &&
                     !body_data.valid,
                     "sensor-to-body conversion must preserve metadata and valid state");
    return passed;
}

static void ResetModel()
{
    memset(g_acc_registers, 0, sizeof(g_acc_registers));
    memset(g_gyro_registers, 0, sizeof(g_gyro_registers));
    memset(g_events, 0, sizeof(g_events));
    memset(g_transfer_lengths, 0, sizeof(g_transfer_lengths));
    g_acc_registers[0x00U] = 0x1EU;
    g_gyro_registers[0x00U] = 0x0FU;
    g_event_count = 0U;
    g_transfer_count = 0U;
    g_fail_transfer_number = 0U;
    g_transfer_length_count = 0U;
    g_acc_cs_low = false;
    g_gyro_cs_low = false;
    g_acc_spi_active = true;
    g_elapsed_ms = 0U;
    g_gyro_quiet_until_ms = 0U;
    g_corrupt_readback = false;
    g_corrupt_accelerometer = false;
    g_corrupt_address = 0U;
    g_corrupt_mask = 0U;
    g_last_gyro_bandwidth_read_value = 0U;
}

static bool ExpectEvent(uint32_t index, EventType type, uint8_t address, uint8_t value)
{
    return g_events[index].type == type &&
           g_events[index].address == address &&
           g_events[index].value == value;
}

static bool ExpectSampleUnchanged(const Bmi088RawSample& sample, const Bmi088RawSample& sentinel)
{
    return sample.acc_x == sentinel.acc_x &&
           sample.acc_y == sentinel.acc_y &&
           sample.acc_z == sentinel.acc_z &&
           sample.gyro_x == sentinel.gyro_x &&
           sample.gyro_y == sentinel.gyro_y &&
           sample.gyro_z == sentinel.gyro_z &&
           sample.temperature_raw == sentinel.temperature_raw &&
           sample.temperature_deg_c == sentinel.temperature_deg_c;
}

static bool ExpectInitializationTrace()
{
    static const RecordedEvent expected_events[] = {
        {EventType::AccSpiReactivation, 0x00U, 0x00U},
        {EventType::AccWrite, 0x7EU, 0xB6U},
        {EventType::Delay, 0x00U, 1U},
        {EventType::AccSpiReactivation, 0x00U, 0x00U},
        {EventType::GyroWrite, 0x14U, 0xB6U},
        {EventType::Delay, 0x00U, 30U},
        {EventType::AccRead, 0x00U, 0x1EU},
        {EventType::AccRead, 0x00U, 0x1EU},
        {EventType::GyroRead, 0x00U, 0x0FU},
        {EventType::AccWrite, 0x7CU, 0x00U},
        {EventType::Delay, 0x00U, 5U},
        {EventType::AccWrite, 0x7DU, 0x04U},
        {EventType::Delay, 0x00U, 5U},
        {EventType::AccWrite, 0x40U, 0x8CU},
        {EventType::Delay, 0x00U, 40U},
        {EventType::AccWrite, 0x41U, 0x01U},
        {EventType::Delay, 0x00U, 1U},
        {EventType::GyroWrite, 0x10U, 0x02U},
        {EventType::Delay, 0x00U, 1U},
        {EventType::GyroWrite, 0x0FU, 0x00U},
        {EventType::Delay, 0x00U, 10U},
        {EventType::AccRead, 0x7CU, 0x00U},
        {EventType::AccRead, 0x7DU, 0x04U},
        {EventType::AccRead, 0x40U, 0x8CU},
        {EventType::AccRead, 0x41U, 0x01U},
        {EventType::GyroRead, 0x10U, 0x82U},
        {EventType::GyroRead, 0x0FU, 0x00U}
    };

    if (g_event_count != sizeof(expected_events) / sizeof(expected_events[0])) {
        return false;
    }
    for (uint32_t index = 0U; index < sizeof(expected_events) / sizeof(expected_events[0]); ++index) {
        if (!ExpectEvent(index, expected_events[index].type,
                         expected_events[index].address, expected_events[index].value)) {
            return false;
        }
    }
    return true;
}

static bool InitializeRejectsReadbackMismatch(
    Class_SpiBus& spi_bus,
    Class_GpioPin& acc_cs,
    Class_GpioPin& gyro_cs,
    bool accelerometer,
    uint8_t address,
    uint8_t mask)
{
    ResetModel();
    Class_Bmi088 bmi088;
    if (bmi088.Bind(&spi_bus, &acc_cs, &gyro_cs) != ResultCode::Ok) {
        return false;
    }
    g_corrupt_readback = true;
    g_corrupt_accelerometer = accelerometer;
    g_corrupt_address = address;
    g_corrupt_mask = mask;
    return bmi088.Initialize(RecordDelayMs) == ResultCode::InvalidData && !bmi088.IsInitialized();
}

int main()
{
    bool passed = true;
    passed &= ExpectSensorToBodyMapping();
    passed &= ExpectBmi088RawToImuConversion();
    ResetModel();

    Class_SpiBus spi_bus;
    Class_GpioPin acc_cs;
    Class_GpioPin gyro_cs;
    Class_Bmi088 bmi088;
    passed &= Expect(spi_bus.Bind(&g_spi_handle) == ResultCode::Ok, "SPI binding must succeed");
    passed &= Expect(acc_cs.Bind(&g_acc_port, kAccPin) == ResultCode::Ok, "ACC CS binding must succeed");
    passed &= Expect(gyro_cs.Bind(&g_gyro_port, kGyroPin) == ResultCode::Ok, "GYRO CS binding must succeed");
    passed &= Expect(bmi088.Bind(&spi_bus, &acc_cs, &gyro_cs) == ResultCode::Ok, "BMI088 binding must succeed");
    g_acc_spi_active = false;
    g_transfer_length_count = 0U;
    uint8_t traced_acc_chip_id = 0U;
    uint8_t traced_gyro_chip_id = 0U;
    Bmi088ProbeTrace probe_trace = {};
    passed &= Expect(
        bmi088.ProbeWithTrace(traced_acc_chip_id, traced_gyro_chip_id, probe_trace) == ResultCode::Ok,
        "traced probe must preserve normal probe behavior");
    passed &= Expect(traced_acc_chip_id == 0x1EU && traced_gyro_chip_id == 0x0FU,
                     "traced probe must decode both chip IDs");
    passed &= Expect(probe_trace.acc_activation_result == ResultCode::Ok &&
                     probe_trace.acc_id_result == ResultCode::Ok &&
                     probe_trace.gyro_id_result == ResultCode::Ok,
                     "traced probe must retain each exact transfer result");
    passed &= Expect(probe_trace.acc_activation_rx[0] == 0U &&
                     probe_trace.acc_activation_rx[1] == 0U &&
                     probe_trace.acc_activation_rx[2] == 0U,
                     "traced probe must retain the initial ACC SPI activation response");
    passed &= Expect(probe_trace.acc_id_rx[0] == 0U &&
                     probe_trace.acc_id_rx[1] == 0U &&
                     probe_trace.acc_id_rx[2] == 0x1EU,
                     "traced probe must retain the exact ACC ID transaction");
    passed &= Expect(probe_trace.gyro_id_rx[0] == 0U && probe_trace.gyro_id_rx[1] == 0x0FU,
                     "traced probe must retain the exact GYRO ID transaction");
    passed &= Expect(g_transfer_length_count == 3U &&
                     g_transfer_lengths[0] == 3U &&
                     g_transfer_lengths[1] == 3U &&
                     g_transfer_lengths[2] == 2U,
                     "traced probe must use the production transaction lengths");
    ResetModel();
    passed &= Expect(!bmi088.IsInitialized(), "new BMI088 must not be initialized");
    Bmi088RawSample pre_init_sample = {};
    passed &= Expect(bmi088.ReadRawSample(pre_init_sample) == ResultCode::NotReady,
                     "sampling before initialization must report NotReady");
    passed &= Expect(bmi088.Initialize(nullptr) == ResultCode::ConfigError,
                     "initialization requires a delay callback");
    g_acc_spi_active = false;
    passed &= Expect(bmi088.Initialize(RecordDelayMs) == ResultCode::Ok,
                     "initialization must activate ACC SPI before issuing reset and configuration writes");
    passed &= Expect(bmi088.IsInitialized(), "successful initialization must latch readiness");

    passed &= Expect(ExpectInitializationTrace(),
                     "initialization must follow the complete reset, probe, configuration, and readback trace");
    passed &= Expect(g_last_gyro_bandwidth_read_value == 0x82U,
                     "gyro bandwidth bit 7 may read back as one without rejecting initialization");
    passed &= Expect(InitializeRejectsReadbackMismatch(spi_bus, acc_cs, gyro_cs, true, 0x7CU, 0x01U),
                     "ACC power config mismatch must be rejected");
    passed &= Expect(InitializeRejectsReadbackMismatch(spi_bus, acc_cs, gyro_cs, true, 0x7DU, 0x01U),
                     "ACC power control mismatch must be rejected");
    passed &= Expect(InitializeRejectsReadbackMismatch(spi_bus, acc_cs, gyro_cs, true, 0x40U, 0x01U),
                     "ACC config mismatch must be rejected");
    passed &= Expect(InitializeRejectsReadbackMismatch(spi_bus, acc_cs, gyro_cs, true, 0x41U, 0x01U),
                     "ACC range mismatch must be rejected");
    passed &= Expect(InitializeRejectsReadbackMismatch(spi_bus, acc_cs, gyro_cs, false, 0x10U, 0x01U),
                     "gyro bandwidth low-bit mismatch must be rejected");
    passed &= Expect(InitializeRejectsReadbackMismatch(spi_bus, acc_cs, gyro_cs, false, 0x0FU, 0x01U),
                     "gyro range mismatch must be rejected");

    ResetModel();
    Class_Bmi088 sample_bmi088;
    passed &= Expect(sample_bmi088.Bind(&spi_bus, &acc_cs, &gyro_cs) == ResultCode::Ok,
                     "sample BMI088 binding must succeed");
    passed &= Expect(sample_bmi088.Initialize(RecordDelayMs) == ResultCode::Ok,
                     "sample BMI088 initialization must succeed");
    g_acc_registers[0x12U] = 0x34U;
    g_acc_registers[0x13U] = 0x12U;
    g_acc_registers[0x14U] = 0xDCU;
    g_acc_registers[0x15U] = 0xFEU;
    g_acc_registers[0x16U] = 0x01U;
    g_acc_registers[0x17U] = 0x80U;
    g_gyro_registers[0x02U] = 0xFFU;
    g_gyro_registers[0x03U] = 0x7FU;
    g_gyro_registers[0x04U] = 0x00U;
    g_gyro_registers[0x05U] = 0x80U;
    g_gyro_registers[0x06U] = 0x00U;
    g_gyro_registers[0x07U] = 0x80U;
    g_acc_registers[0x22U] = 0xFFU;
    g_acc_registers[0x23U] = 0x00U;
    g_transfer_length_count = 0U;
    Bmi088RawSample sample = {};
    passed &= Expect(sample_bmi088.ReadRawSample(sample) == ResultCode::Ok,
                     "complete raw sample must succeed");
    passed &= Expect(sample.acc_x == 0x1234, "ACC X must decode little-endian");
    passed &= Expect(sample.acc_y == static_cast<int16_t>(0xFEDC), "ACC Y must keep sign");
    passed &= Expect(sample.acc_z == static_cast<int16_t>(0x8001), "ACC Z must keep sign");
    passed &= Expect(sample.gyro_x == 0x7FFF, "GYRO X must decode little-endian");
    passed &= Expect(sample.gyro_y == static_cast<int16_t>(0x8000), "GYRO Y must keep sign");
    passed &= Expect(sample.gyro_z == static_cast<int16_t>(0x8000), "GYRO Z must keep sign");
    passed &= Expect(sample.temperature_raw == -8, "temperature must sign-extend 11 bits");
    passed &= Expect(sample.temperature_deg_c == 22.0f,
                     "temperature conversion must use 0.125 C/LSB + 23 C");
    passed &= Expect(g_transfer_length_count == 3U, "sampling must use three transactions");
    passed &= Expect(g_transfer_lengths[0] == 8U, "ACC XYZ transaction must include a dummy byte");
    passed &= Expect(g_transfer_lengths[1] == 7U, "GYRO XYZ transaction must omit a dummy byte");
    passed &= Expect(g_transfer_lengths[2] == 4U, "temperature transaction must include a dummy byte");

    g_acc_registers[0x22U] = 0x00U;
    g_acc_registers[0x23U] = 0x00U;
    passed &= Expect(sample_bmi088.ReadRawSample(sample) == ResultCode::Ok,
                     "zero temperature sample must succeed");
    passed &= Expect(sample.temperature_raw == 0 && sample.temperature_deg_c == 23.0f,
                     "zero temperature must decode to 23 C");
    g_acc_registers[0x22U] = 0x01U;
    passed &= Expect(sample_bmi088.ReadRawSample(sample) == ResultCode::Ok,
                     "positive temperature sample must succeed");
    passed &= Expect(sample.temperature_raw == 8 && sample.temperature_deg_c == 24.0f,
                     "positive temperature must use the documented 11-bit format");

    Bmi088RawSample sentinel = {11, 12, 13, 14, 15, 16, 17, 18.0f};
    for (uint32_t failed_transfer = 1U; failed_transfer <= 3U; ++failed_transfer) {
        sample = sentinel;
        g_transfer_count = 0U;
        g_fail_transfer_number = failed_transfer;
        passed &= Expect(sample_bmi088.ReadRawSample(sample) == ResultCode::HardwareError,
                         "a failed sample transfer must report the SPI error");
        passed &= Expect(!g_acc_cs_low && !g_gyro_cs_low,
                         "a failed sample transfer must restore both chip selects high");
        passed &= Expect(ExpectSampleUnchanged(sample, sentinel),
                         "each failed sample transfer must leave output unchanged");
    }

    if (!passed) {
        return 1;
    }
    printf("BMI088 device behavior checks passed.\n");
    return 0;
}
