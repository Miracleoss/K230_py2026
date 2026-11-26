#include "Driver/drv_uart_dma.h"

Class_UartDma::Class_UartDma()
    : uart_(nullptr),
      rx_buffer_(nullptr),
      rx_capacity_(0U),
      rx_read_position_(0U),
      rx_overflow_count_(0U),
      rx_error_count_(0U),
      rx_restart_count_(0U),
      rx_last_error_(HAL_UART_ERROR_NONE),
      rx_restart_pending_(false),
      rx_started_(false)
{
}

ResultCode Class_UartDma::Bind(UART_HandleTypeDef* uart)
{
    if (uart == nullptr) {
        return ResultCode::ConfigError;
    }

    uart_ = uart;
    return ResultCode::Ok;
}

bool Class_UartDma::IsBound() const
{
    return uart_ != nullptr;
}

ResultCode Class_UartDma::StartReceiveCircular(uint8_t* buffer, uint16_t capacity)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    if (buffer == nullptr || capacity == 0U) {
        return ResultCode::InvalidData;
    }

    if (rx_started_) {
        return ResultCode::NotReady;
    }

    rx_buffer_ = buffer;
    rx_capacity_ = capacity;
    rx_read_position_ = 0U;
    rx_overflow_count_ = 0U;
    rx_error_count_ = 0U;
    rx_restart_count_ = 0U;
    rx_last_error_ = HAL_UART_ERROR_NONE;
    rx_restart_pending_ = false;

    const HAL_StatusTypeDef status = StartHalReceive();
    return MapHalStatus(status);
}

ResultCode Class_UartDma::ServiceReceive()
{
    if (!IsBound() || rx_buffer_ == nullptr || rx_capacity_ == 0U) {
        return ResultCode::ConfigError;
    }

    if (!rx_restart_pending_) {
        return rx_started_ ? ResultCode::Ok : ResultCode::NotReady;
    }

    rx_restart_pending_ = false;
    rx_started_ = false;
    const HAL_StatusTypeDef abort_status = HAL_UART_AbortReceive(uart_);
    if (abort_status != HAL_OK) {
        rx_restart_pending_ = true;
        return MapHalStatus(abort_status);
    }

    rx_read_position_ = 0U;
    const HAL_StatusTypeDef start_status = StartHalReceive();
    if (start_status != HAL_OK) {
        rx_restart_pending_ = true;
        return MapHalStatus(start_status);
    }

    ++rx_restart_count_;
    return ResultCode::InvalidData;
}

ResultCode Class_UartDma::SendAsync(uint8_t* data, uint16_t length)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    if (data == nullptr || length == 0U) {
        return ResultCode::InvalidData;
    }

    return MapHalStatus(HAL_UART_Transmit_DMA(uart_, data, length));
}

void Class_UartDma::OnRxErrorIsr(uint32_t error_code)
{
    rx_last_error_ = error_code;
    ++rx_error_count_;
    if ((error_code & HAL_UART_ERROR_ORE) != 0U) {
        ++rx_overflow_count_;
    }
    rx_started_ = false;
    rx_restart_pending_ = true;
}

uint32_t Class_UartDma::Available() const
{
    if (!rx_started_) {
        return 0U;
    }

    const uint16_t write_position = GetDmaWritePosition();
    if (write_position >= rx_read_position_) {
        return static_cast<uint32_t>(write_position - rx_read_position_);
    }

    return static_cast<uint32_t>(rx_capacity_ - rx_read_position_ + write_position);
}

ResultCode Class_UartDma::ReadByte(uint8_t& byte)
{
    if (!rx_started_) {
        return ResultCode::NotReady;
    }

    if (Available() == 0U) {
        return ResultCode::NotReady;
    }

    byte = rx_buffer_[rx_read_position_];
    ++rx_read_position_;
    if (rx_read_position_ == rx_capacity_) {
        rx_read_position_ = 0U;
    }
    return ResultCode::Ok;
}

uint32_t Class_UartDma::GetRxOverflowCount() const
{
    return rx_overflow_count_;
}

uint32_t Class_UartDma::GetRxErrorCount() const
{
    return rx_error_count_;
}

uint32_t Class_UartDma::GetRxRestartCount() const
{
    return rx_restart_count_;
}

uint32_t Class_UartDma::GetLastRxError() const
{
    return rx_last_error_;
}

HAL_StatusTypeDef Class_UartDma::StartHalReceive()
{
    rx_started_ = true;
    const HAL_StatusTypeDef status = HAL_UART_Receive_DMA(uart_, rx_buffer_, rx_capacity_);
    if (status != HAL_OK) {
        rx_started_ = false;
        return status;
    }

    __HAL_DMA_DISABLE_IT(uart_->hdmarx, DMA_IT_HT | DMA_IT_TC);
    return HAL_OK;
}

uint16_t Class_UartDma::GetDmaWritePosition() const
{
    const uint32_t remaining = __HAL_DMA_GET_COUNTER(uart_->hdmarx);
    if (remaining >= rx_capacity_) {
        return 0U;
    }

    return static_cast<uint16_t>(rx_capacity_ - remaining);
}

ResultCode Class_UartDma::MapHalStatus(HAL_StatusTypeDef status) const
{
    switch (status) {
    case HAL_OK:
        return ResultCode::Ok;
    case HAL_TIMEOUT:
        return ResultCode::Timeout;
    case HAL_BUSY:
        return ResultCode::NotReady;
    case HAL_ERROR:
    default:
        return ResultCode::HardwareError;
    }
}
