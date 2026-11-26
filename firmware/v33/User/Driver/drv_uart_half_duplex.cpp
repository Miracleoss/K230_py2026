#include "Driver/drv_uart_half_duplex.h"

Class_UartHalfDuplex::Class_UartHalfDuplex()
    : uart_(nullptr),
      rx_buffer_(nullptr),
      rx_capacity_(0U),
      rx_write_position_(0U),
      rx_read_position_(0U),
      rx_byte_(0U),
      tx_buffer_(),
      is_tx_busy_(false),
      tx_count_(0U),
      rx_byte_count_(0U),
      rx_overflow_count_(0U),
      rx_error_count_(0U),
      rx_last_error_(HAL_UART_ERROR_NONE)
{
}

ResultCode Class_UartHalfDuplex::Bind(UART_HandleTypeDef* uart)
{
    if (uart == nullptr) {
        return ResultCode::ConfigError;
    }

    uart_ = uart;
    return ResultCode::Ok;
}

bool Class_UartHalfDuplex::IsBound() const
{
    return uart_ != nullptr;
}

ResultCode Class_UartHalfDuplex::StartReceive(uint8_t* buffer, uint16_t capacity)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    if (buffer == nullptr || capacity == 0U) {
        return ResultCode::InvalidData;
    }

    rx_buffer_ = buffer;
    rx_capacity_ = capacity;
    rx_write_position_ = 0U;
    rx_read_position_ = 0U;
    is_tx_busy_ = false;

    HAL_HalfDuplex_EnableReceiver(uart_);
    const HAL_StatusTypeDef status = HAL_UART_Receive_IT(uart_, &rx_byte_, 1U);
    return MapHalStatus(status);
}

ResultCode Class_UartHalfDuplex::SendAsync(const uint8_t* data, uint16_t length)
{
    if (!IsBound()) {
        return ResultCode::ConfigError;
    }

    if (data == nullptr || length == 0U || length > sizeof(tx_buffer_)) {
        return ResultCode::InvalidData;
    }

    if (is_tx_busy_) {
        return ResultCode::NotReady;
    }

    // 中止正在进行的接收过程
    HAL_UART_AbortReceive(uart_);

    // 切换单线半双工至发送态（使能 TE，关闭 RE）
    HAL_HalfDuplex_EnableTransmitter(uart_);

    for (uint16_t i = 0U; i < length; ++i) {
        tx_buffer_[i] = data[i];
    }
    is_tx_busy_ = true;

    const HAL_StatusTypeDef status = HAL_UART_Transmit_IT(uart_, tx_buffer_, length);
    if (status != HAL_OK) {
        is_tx_busy_ = false;
        HAL_HalfDuplex_EnableReceiver(uart_);
        HAL_UART_Receive_IT(uart_, &rx_byte_, 1U);
        return MapHalStatus(status);
    }

    ++tx_count_;
    return ResultCode::Ok;
}

bool Class_UartHalfDuplex::IsTxBusy() const
{
    return is_tx_busy_;
}

uint32_t Class_UartHalfDuplex::Available() const
{
    if (rx_buffer_ == nullptr || rx_capacity_ == 0U) {
        return 0U;
    }

    const uint16_t write_pos = rx_write_position_;
    const uint16_t read_pos = rx_read_position_;
    if (write_pos >= read_pos) {
        return static_cast<uint32_t>(write_pos - read_pos);
    }

    return static_cast<uint32_t>(rx_capacity_ - read_pos + write_pos);
}

ResultCode Class_UartHalfDuplex::ReadByte(uint8_t& byte)
{
    if (Available() == 0U) {
        return ResultCode::NotReady;
    }

    byte = rx_buffer_[rx_read_position_];
    uint16_t next_read = rx_read_position_ + 1U;
    if (next_read >= rx_capacity_) {
        next_read = 0U;
    }
    rx_read_position_ = next_read;
    return ResultCode::Ok;
}

void Class_UartHalfDuplex::FlushRx()
{
    rx_read_position_ = rx_write_position_;
}

void Class_UartHalfDuplex::OnTxCpltIsr()
{
    is_tx_busy_ = false;

    if (uart_ == nullptr) {
        return;
    }

    // 发送完成（TC 标志置位），立即切回接收态（使能 RE，关闭 TE）
    HAL_HalfDuplex_EnableReceiver(uart_);

    // 清除可能因回环产生的残余标志
    __HAL_UART_CLEAR_FLAG(uart_, UART_FLAG_RXNE | UART_FLAG_TC);
    volatile uint32_t dummy = uart_->Instance->DR;
    (void)dummy;

    // 重新挂起 1 字节接收中断
    if (rx_buffer_ != nullptr && rx_capacity_ > 0U) {
        HAL_UART_Receive_IT(uart_, &rx_byte_, 1U);
    }
}

void Class_UartHalfDuplex::OnRxCpltIsr()
{
    ++rx_byte_count_;

    if (rx_buffer_ != nullptr && rx_capacity_ > 0U) {
        uint16_t next_write = rx_write_position_ + 1U;
        if (next_write >= rx_capacity_) {
            next_write = 0U;
        }

        if (next_write == rx_read_position_) {
            ++rx_overflow_count_;
        } else {
            rx_buffer_[rx_write_position_] = rx_byte_;
            rx_write_position_ = next_write;
        }
    }

    if (uart_ != nullptr && !is_tx_busy_) {
        HAL_UART_Receive_IT(uart_, &rx_byte_, 1U);
    }
}

void Class_UartHalfDuplex::OnErrorIsr(uint32_t error_code)
{
    rx_last_error_ = error_code;
    ++rx_error_count_;
    if ((error_code & HAL_UART_ERROR_ORE) != 0U) {
        ++rx_overflow_count_;
    }

    if (!is_tx_busy_ && uart_ != nullptr) {
        __HAL_UART_CLEAR_OREFLAG(uart_);
        __HAL_UART_CLEAR_NEFLAG(uart_);
        __HAL_UART_CLEAR_FEFLAG(uart_);
        __HAL_UART_CLEAR_PEFLAG(uart_);
        volatile uint32_t dummy = uart_->Instance->DR;
        (void)dummy;
        HAL_HalfDuplex_EnableReceiver(uart_);
        HAL_UART_Receive_IT(uart_, &rx_byte_, 1U);
    }
}

uint32_t Class_UartHalfDuplex::GetTxCount() const
{
    return tx_count_;
}

uint32_t Class_UartHalfDuplex::GetRxByteCount() const
{
    return rx_byte_count_;
}

uint32_t Class_UartHalfDuplex::GetRxOverflowCount() const
{
    return rx_overflow_count_;
}

uint32_t Class_UartHalfDuplex::GetRxErrorCount() const
{
    return rx_error_count_;
}

uint32_t Class_UartHalfDuplex::GetLastRxError() const
{
    return rx_last_error_;
}

ResultCode Class_UartHalfDuplex::MapHalStatus(HAL_StatusTypeDef status) const
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

