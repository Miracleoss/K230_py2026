#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Device/dev_host_serial.h"

static HAL_StatusTypeDef receive_status = HAL_OK;
static HAL_StatusTypeDef abort_status = HAL_OK;
static HAL_StatusTypeDef transmit_status = HAL_OK;
static uint8_t* last_tx_data = nullptr;
static uint16_t last_tx_length = 0U;

extern "C" HAL_StatusTypeDef HAL_UART_Receive_DMA(
    UART_HandleTypeDef* uart,
    uint8_t*,
    uint16_t size)
{
    if (receive_status == HAL_OK) {
        uart->hdmarx->Instance->NDTR = size;
    }
    return receive_status;
}

extern "C" HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*)
{
    return abort_status;
}

extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(
    UART_HandleTypeDef*,
    uint8_t* data,
    uint16_t length)
{
    if (transmit_status == HAL_OK) {
        last_tx_data = data;
        last_tx_length = length;
    }
    return transmit_status;
}

static bool Expect(bool condition, const char* message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        return false;
    }
    return true;
}

static void WriteCircular(
    uint8_t* buffer,
    uint16_t capacity,
    uint32_t absolute_position,
    const uint8_t* data,
    uint16_t length)
{
    for (uint16_t index = 0U; index < length; ++index) {
        buffer[(absolute_position + index) % capacity] = data[index];
    }
}

static void SetWritePosition(DMA_Stream_TypeDef& stream, uint16_t capacity, uint16_t position)
{
    stream.NDTR = position == 0U ? capacity : static_cast<uint32_t>(capacity - position);
}

int main()
{
    bool passed = true;
    DMA_Stream_TypeDef dma_stream = {};
    DMA_HandleTypeDef dma_handle = {};
    dma_handle.Instance = &dma_stream;
    UART_HandleTypeDef uart_handle = {};
    uart_handle.hdmarx = &dma_handle;
    uint8_t rx_buffer[256] = {};
    Class_UartDma uart;
    passed &= Expect(uart.Bind(&uart_handle) == ResultCode::Ok, "UART bind must succeed");
    passed &= Expect(uart.StartReceiveCircular(rx_buffer, sizeof(rx_buffer)) == ResultCode::Ok,
                     "UART circular receive must start");

    Class_HostSerial host;
    HostOverloadCommand command;
    passed &= Expect(host.Poll() == ResultCode::ConfigError, "poll before bind must reject configuration");
    passed &= Expect(host.TakeLatestOverload(command) == ResultCode::NotReady,
                     "no command may be published before bind");
    passed &= Expect(host.Bind(&uart) == ResultCode::Ok, "host serial bind must succeed");
    passed &= Expect(host.Poll() == ResultCode::NotReady, "an empty UART ring must be not ready");

    const uint8_t valid_frame[kHostControlFrameSize] = {
        0xAAU, 0x55U,
        0xB5U, 0xFFU,
        0x96U, 0x00U,
        0x01U, 0xFFU,
        0x00U, 0x00U,
        0x3EU, 0x9DU,
        0x0DU, 0x0AU
    };
    const uint8_t noise[] = {0x00U, 0xAAU, 0x01U};
    WriteCircular(rx_buffer, sizeof(rx_buffer), 0U, noise, sizeof(noise));
    WriteCircular(rx_buffer, sizeof(rx_buffer), sizeof(noise), valid_frame, sizeof(valid_frame));
    SetWritePosition(
        dma_stream,
        sizeof(rx_buffer),
        static_cast<uint16_t>(sizeof(noise) + sizeof(valid_frame)));
    passed &= Expect(host.Poll() == ResultCode::Ok, "poll must consume an available valid frame");
    passed &= Expect(host.TakeLatestOverload(command) == ResultCode::Ok,
                     "a valid frame must publish a command");
    passed &= Expect(command.valid && command.is_launched == 0xB5U && command.is_base_green_detected == 0xFFU &&
                         command.yaw_overload_g == 1.50f && command.pitch_overload_g == -2.55f,
                     "published overload values must match the wire data");
    passed &= Expect(host.TakeLatestOverload(command) == ResultCode::NotReady,
                     "taking a command must clear the new-command flag");

    uint8_t invalid_frame[kHostControlFrameSize] = {};
    memcpy(invalid_frame, valid_frame, sizeof(invalid_frame));
    invalid_frame[10] ^= 0x01U;
    const uint32_t position_after_valid = sizeof(noise) + sizeof(valid_frame);
    WriteCircular(rx_buffer, sizeof(rx_buffer), position_after_valid, invalid_frame, sizeof(invalid_frame));
    SetWritePosition(
        dma_stream,
        sizeof(rx_buffer),
        static_cast<uint16_t>(position_after_valid + sizeof(invalid_frame)));
    passed &= Expect(host.Poll() == ResultCode::Ok, "poll must consume a malformed frame without blocking");
    passed &= Expect(host.TakeLatestOverload(command) == ResultCode::NotReady,
                     "a bad CRC must not publish a command");
    passed &= Expect(host.GetCrcErrorCount() == 1U, "host serial must expose parser CRC diagnostics");

    const uint32_t partial_position = position_after_valid + sizeof(invalid_frame);
    WriteCircular(rx_buffer, sizeof(rx_buffer), partial_position, valid_frame, 5U);
    SetWritePosition(
        dma_stream,
        sizeof(rx_buffer),
        static_cast<uint16_t>(partial_position + 5U));
    passed &= Expect(host.Poll() == ResultCode::Ok, "a partial frame must be consumed without publishing");
    uart.OnRxErrorIsr(HAL_UART_ERROR_ORE);
    passed &= Expect(host.Poll() == ResultCode::InvalidData,
                     "UART recovery must reset partial parser state and report invalid data");
    passed &= Expect(host.GetRxOverflowCount() == 1U, "host serial must expose UART overflow diagnostics");

    WriteCircular(rx_buffer, sizeof(rx_buffer), 0U, valid_frame, sizeof(valid_frame));
    SetWritePosition(dma_stream, sizeof(rx_buffer), sizeof(valid_frame));
    passed &= Expect(host.Poll() == ResultCode::Ok, "parser must recover after UART restart");
    passed &= Expect(host.TakeLatestOverload(command) == ResultCode::Ok,
                     "a valid frame after overflow must publish");

    ImuData imu;
    imu.specific_force_x_mps2 = 9.81f;
    imu.specific_force_y_mps2 = 0.0f;
    imu.specific_force_z_mps2 = -9.81f;
    imu.gyro_x_radps = 0.0f;
    imu.gyro_y_radps = 0.0f;
    imu.gyro_z_radps = 0.0f;
    imu.valid = true;

    BodyControlCommand body_command;
    body_command.pitch = 0.25f;
    body_command.yaw = -0.50f;
    body_command.valid = true;

    Class_HostSerial unbound_host;
    passed &= Expect(unbound_host.SendImuData(imu, body_command, 0.0f, 0.0f) == ResultCode::ConfigError,
                     "send before bind must reject configuration");

    transmit_status = HAL_OK;
    passed &= Expect(host.SendImuData(imu, body_command, 0.0f, 0.0f) == ResultCode::Ok,
                     "first IMU DMA send must succeed");
    passed &= Expect(last_tx_data != nullptr && last_tx_length == kHostImuFrameSize,
                     "IMU DMA send must use the complete fixed frame");
    passed &= Expect(last_tx_data[14] == 0x9FU && last_tx_data[15] == 0x40U,
                     "IMU DMA send must carry mapped P/Y command bytes");
    uint8_t* first_tx_data = last_tx_data;
    uint8_t first_tx_snapshot[kHostImuFrameSize] = {};
    memcpy(first_tx_snapshot, first_tx_data, sizeof(first_tx_snapshot));

    imu.specific_force_x_mps2 = 1.25f;
    transmit_status = HAL_BUSY;
    passed &= Expect(host.SendImuData(imu, body_command, 0.0f, 0.0f) == ResultCode::NotReady,
                     "HAL busy must propagate as not ready");
    passed &= Expect(memcmp(first_tx_data, first_tx_snapshot, sizeof(first_tx_snapshot)) == 0,
                     "a busy retry must not overwrite the in-flight DMA buffer");

    transmit_status = HAL_OK;
    passed &= Expect(host.SendImuData(imu, body_command, 0.0f, 0.0f) == ResultCode::Ok,
                     "retry after DMA ready must succeed");
    passed &= Expect(last_tx_data != first_tx_data, "successful consecutive sends must alternate DMA buffers");
    passed &= Expect(last_tx_data[2] == 0x7DU && last_tx_data[3] == 0x00U,
                     "the retried frame must contain the new acceleration value");

    Class_HostSerial unbound_vofa_host;
    passed &= Expect(unbound_vofa_host.SendVofaJustFloatData(imu) == ResultCode::ConfigError,
                     "VOFA send before bind must reject configuration");

    ImuData vofa_imu;
    vofa_imu.specific_force_x_mps2 = 1.0f;
    vofa_imu.specific_force_y_mps2 = 2.0f;
    vofa_imu.specific_force_z_mps2 = 3.0f;
    vofa_imu.gyro_x_radps = 4.0f;
    vofa_imu.gyro_y_radps = 5.0f;
    vofa_imu.gyro_z_radps = 6.0f;
    vofa_imu.valid = true;

    transmit_status = HAL_OK;
    passed &= Expect(host.SendVofaJustFloatData(vofa_imu) == ResultCode::Ok,
                     "first VOFA JustFloat DMA send must succeed");
    passed &= Expect(last_tx_data != nullptr && last_tx_length == kVofaJustFloatFrameSize,
                     "VOFA DMA send must use the complete JustFloat frame");
    uint8_t* first_vofa_tx_data = last_tx_data;
    uint8_t first_vofa_tx_snapshot[kVofaJustFloatFrameSize] = {};
    memcpy(first_vofa_tx_snapshot, first_vofa_tx_data, sizeof(first_vofa_tx_snapshot));

    vofa_imu.specific_force_x_mps2 = 7.0f;
    transmit_status = HAL_BUSY;
    passed &= Expect(host.SendVofaJustFloatData(vofa_imu) == ResultCode::NotReady,
                     "VOFA HAL busy must propagate as not ready");
    passed &= Expect(memcmp(first_vofa_tx_data, first_vofa_tx_snapshot,
                            sizeof(first_vofa_tx_snapshot)) == 0,
                     "a VOFA busy retry must not overwrite the in-flight DMA buffer");

    transmit_status = HAL_OK;
    passed &= Expect(host.SendVofaJustFloatData(vofa_imu) == ResultCode::Ok,
                     "VOFA retry after DMA ready must succeed");
    passed &= Expect(last_tx_data != first_vofa_tx_data,
                     "successful VOFA sends must alternate their DMA buffers");

    if (!passed) {
        return 1;
    }

    printf("Host serial device checks passed.\n");
    return 0;
}
