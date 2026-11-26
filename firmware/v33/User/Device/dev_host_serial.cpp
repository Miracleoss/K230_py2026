#include "Device/dev_host_serial.h"

Class_HostSerial::Class_HostSerial()
    : uart_(nullptr),
      parser_(),
      latest_command_(),
      command_ready_(false),
      tx_buffers_(),
      next_tx_buffer_(0U),
      vofa_tx_buffers_(),
      next_vofa_tx_buffer_(0U)
{
}

ResultCode Class_HostSerial::Bind(Class_UartDma* uart)
{
    if (uart == nullptr || !uart->IsBound()) {
        return ResultCode::ConfigError;
    }

    uart_ = uart;
    parser_.Reset();
    latest_command_ = HostOverloadCommand();
    command_ready_ = false;
    next_tx_buffer_ = 0U;
    next_vofa_tx_buffer_ = 0U;
    return ResultCode::Ok;
}

ResultCode Class_HostSerial::Poll()
{
    if (uart_ == nullptr) {
        return ResultCode::ConfigError;
    }

    const ResultCode service_result = uart_->ServiceReceive();
    if (service_result == ResultCode::InvalidData) {
        parser_.Reset();
        return ResultCode::InvalidData;
    }
    if (service_result != ResultCode::Ok) {
        return service_result;
    }

    bool consumed_byte = false;
    while (uart_->Available() > 0U) {
        uint8_t byte = 0U;
        const ResultCode read_result = uart_->ReadByte(byte);
        if (read_result != ResultCode::Ok) {
            if (read_result == ResultCode::InvalidData) {
                parser_.Reset();
            }
            return read_result;
        }

        consumed_byte = true;
        HostOverloadCommand decoded_command;
        if (parser_.Feed(byte, decoded_command)) {
            latest_command_ = decoded_command;
            command_ready_ = true;
        }
    }

    return consumed_byte ? ResultCode::Ok : ResultCode::NotReady;
}

ResultCode Class_HostSerial::TakeLatestOverload(HostOverloadCommand& command)
{
    if (!command_ready_) {
        return ResultCode::NotReady;
    }

    command = latest_command_;
    command_ready_ = false;
    return ResultCode::Ok;
}

ResultCode Class_HostSerial::SendImuData(const ImuData& imu, const BodyControlCommand& command,
                                         float overload_yaw_rate_radps, float overload_pitch_rate_radps)
{
    if (uart_ == nullptr) {
        return ResultCode::ConfigError;
    }

    uint8_t* const frame = tx_buffers_[next_tx_buffer_];
    const ResultCode encode_result =
        HostProtocolEncodeImuFrame(imu, command, overload_yaw_rate_radps,
                                   overload_pitch_rate_radps, frame, kHostImuFrameSize);
    if (encode_result != ResultCode::Ok) {
        return encode_result;
    }

    const ResultCode send_result = uart_->SendAsync(frame, kHostImuFrameSize);
    if (send_result == ResultCode::Ok) {
        next_tx_buffer_ ^= 1U;
    }
    return send_result;
}

ResultCode Class_HostSerial::SendVofaJustFloatData(const ImuData& imu)
{
    if (uart_ == nullptr) {
        return ResultCode::ConfigError;
    }

    uint8_t* const frame = vofa_tx_buffers_[next_vofa_tx_buffer_];
    const ResultCode encode_result =
        VofaProtocolEncodeJustFloatFrame(imu, frame, kVofaJustFloatFrameSize);
    if (encode_result != ResultCode::Ok) {
        return encode_result;
    }

    const ResultCode send_result = uart_->SendAsync(frame, kVofaJustFloatFrameSize);
    if (send_result == ResultCode::Ok) {
        next_vofa_tx_buffer_ ^= 1U;
    }
    return send_result;
}

uint32_t Class_HostSerial::GetCrcErrorCount() const
{
    return parser_.GetCrcErrorCount();
}

uint32_t Class_HostSerial::GetFormatErrorCount() const
{
    return parser_.GetFormatErrorCount();
}

uint32_t Class_HostSerial::GetRxOverflowCount() const
{
    return uart_ == nullptr ? 0U : uart_->GetRxOverflowCount();
}
