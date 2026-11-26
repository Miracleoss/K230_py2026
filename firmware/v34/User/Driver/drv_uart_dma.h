#ifndef USER_DRIVER_DRV_UART_DMA_H
#define USER_DRIVER_DRV_UART_DMA_H

#include <stdint.h>

#include "Common/common_types.h"
#include "stm32f4xx_hal.h"

class Class_UartDma {
public:
    Class_UartDma();

    ResultCode Bind(UART_HandleTypeDef* uart);
    bool IsBound() const;

    ResultCode StartReceiveCircular(uint8_t* buffer, uint16_t capacity);
    ResultCode ServiceReceive();
    ResultCode SendAsync(uint8_t* data, uint16_t length);
    void OnRxErrorIsr(uint32_t error_code);
    uint32_t Available() const;
    ResultCode ReadByte(uint8_t& byte);
    uint32_t GetRxOverflowCount() const;
    uint32_t GetRxErrorCount() const;
    uint32_t GetRxRestartCount() const;
    uint32_t GetLastRxError() const;

private:
    HAL_StatusTypeDef StartHalReceive();
    uint16_t GetDmaWritePosition() const;
    ResultCode MapHalStatus(HAL_StatusTypeDef status) const;

    UART_HandleTypeDef* uart_;
    uint8_t* rx_buffer_;
    uint16_t rx_capacity_;
    uint16_t rx_read_position_;
    volatile uint32_t rx_overflow_count_;
    volatile uint32_t rx_error_count_;
    uint32_t rx_restart_count_;
    volatile uint32_t rx_last_error_;
    volatile bool rx_restart_pending_;
    volatile bool rx_started_;
};

#endif /* USER_DRIVER_DRV_UART_DMA_H */
