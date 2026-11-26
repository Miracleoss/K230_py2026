#include <stdint.h>
#include <stdio.h>

#include "Driver/drv_uart_dma.h"

static HAL_StatusTypeDef receive_status = HAL_OK;
static HAL_StatusTypeDef abort_status = HAL_OK;
static HAL_StatusTypeDef transmit_status = HAL_OK;
static uint32_t receive_start_count = 0U;
static uint32_t receive_abort_count = 0U;

extern "C" HAL_StatusTypeDef HAL_UART_Receive_DMA(
    UART_HandleTypeDef* uart,
    uint8_t*,
    uint16_t size)
{
    ++receive_start_count;
    if (receive_status == HAL_OK) {
        uart->hdmarx->Instance->NDTR = size;
    }
    return receive_status;
}

extern "C" HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*)
{
    ++receive_abort_count;
    return abort_status;
}

extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(
    UART_HandleTypeDef*,
    uint8_t*,
    uint16_t)
{
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

    uint8_t buffer[256] = {};
    buffer[0] = 0xEEU;
    buffer[1] = 0x05U;

    Class_UartDma uart;
    passed &= Expect(uart.Bind(&uart_handle) == ResultCode::Ok, "UART bind must succeed");
    passed &= Expect(uart.StartReceiveCircular(buffer, sizeof(buffer)) == ResultCode::Ok,
                     "circular DMA receive must start");
    passed &= Expect(receive_start_count == 1U, "HAL receive must start exactly once");
    passed &= Expect(
        (dma_stream.disabled_interrupts & (DMA_IT_HT | DMA_IT_TC)) == (DMA_IT_HT | DMA_IT_TC),
        "polling RX must disable DMA half and complete interrupts");
    passed &= Expect(uart.StartReceiveCircular(buffer, sizeof(buffer)) == ResultCode::NotReady,
                     "a running receive must reject duplicate start");

    SetWritePosition(dma_stream, sizeof(buffer), 2U);
    passed &= Expect(uart.Available() == 2U, "NDTR 254 must expose two received bytes");
    uint8_t byte = 0U;
    passed &= Expect(uart.ReadByte(byte) == ResultCode::Ok && byte == 0xEEU,
                     "the first DMA byte must be readable");
    passed &= Expect(uart.ReadByte(byte) == ResultCode::Ok && byte == 0x05U,
                     "the second DMA byte must be readable");
    passed &= Expect(uart.Available() == 0U, "consumed bytes must not be repeated");

    SetWritePosition(dma_stream, sizeof(buffer), 254U);
    for (uint16_t index = 2U; index < 254U; ++index) {
        buffer[index] = static_cast<uint8_t>(index);
        passed &= Expect(uart.ReadByte(byte) == ResultCode::Ok,
                         "bytes before wrap must remain readable");
    }
    buffer[254U] = 0xA0U;
    buffer[255U] = 0xB0U;
    buffer[0U] = 0xC0U;
    buffer[1U] = 0xD0U;
    SetWritePosition(dma_stream, sizeof(buffer), 2U);
    const uint8_t expected_wrap[] = {0xA0U, 0xB0U, 0xC0U, 0xD0U};
    for (uint16_t index = 0U; index < sizeof(expected_wrap); ++index) {
        passed &= Expect(
            uart.ReadByte(byte) == ResultCode::Ok && byte == expected_wrap[index],
            "DMA bytes must preserve order across the ring boundary");
    }
    passed &= Expect(uart.ReadByte(byte) == ResultCode::NotReady,
                     "equal DMA and read positions must be empty");

    uart.OnRxErrorIsr(HAL_UART_ERROR_ORE);
    passed &= Expect(uart.Available() == 0U, "faulted receive must not expose stale bytes");
    passed &= Expect(uart.GetRxErrorCount() == 1U, "UART errors must be counted");
    passed &= Expect(uart.GetRxOverflowCount() == 1U, "UART overrun must be counted");
    passed &= Expect(uart.GetLastRxError() == HAL_UART_ERROR_ORE,
                     "the latest HAL UART error must be retained");
    passed &= Expect(uart.ServiceReceive() == ResultCode::InvalidData,
                     "successful recovery must report a stream discontinuity");
    passed &= Expect(receive_abort_count == 1U && receive_start_count == 2U,
                     "main-context recovery must abort and restart receive exactly once");
    passed &= Expect(uart.GetRxRestartCount() == 1U, "successful recovery must be counted");
    passed &= Expect(uart.ServiceReceive() == ResultCode::Ok,
                     "healthy receive service must be idempotent");

    uart.OnRxErrorIsr(0x20U);
    receive_status = HAL_ERROR;
    passed &= Expect(uart.ServiceReceive() == ResultCode::HardwareError,
                     "failed restart must surface the HAL failure");
    receive_status = HAL_OK;
    passed &= Expect(uart.ServiceReceive() == ResultCode::InvalidData,
                     "a failed restart must remain pending for the next service call");
    passed &= Expect(uart.GetRxRestartCount() == 2U,
                     "only successful receive restarts must be counted");

    if (!passed) {
        return 1;
    }

    printf("UART circular DMA polling checks passed.\n");
    return 0;
}
