#ifndef TOOLS_TEST_SUPPORT_STM32F4XX_HAL_H
#define TOOLS_TEST_SUPPORT_STM32F4XX_HAL_H

#include <stdint.h>

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct {
    uint32_t NDTR;
    uint32_t disabled_interrupts;
} DMA_Stream_TypeDef;

typedef struct {
    DMA_Stream_TypeDef* Instance;
} DMA_HandleTypeDef;

typedef struct {
    DMA_HandleTypeDef* hdmarx;
    uint32_t ErrorCode;
} UART_HandleTypeDef;

#define DMA_IT_TC 0x00000010U
#define DMA_IT_HT 0x00000008U
#define HAL_UART_ERROR_NONE 0x00000000U
#define HAL_UART_ERROR_ORE 0x00000008U

#define __HAL_DMA_GET_COUNTER(handle) ((handle)->Instance->NDTR)
#define __HAL_DMA_DISABLE_IT(handle, interrupt) \
    ((handle)->Instance->disabled_interrupts |= (interrupt))

typedef struct {
    uint32_t unused;
} SPI_HandleTypeDef;

typedef struct {
    uint32_t unused;
} GPIO_TypeDef;

typedef enum {
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET = 1U
} GPIO_PinState;

typedef struct {
    struct {
        uint32_t Period;
    } Init;
    uint32_t compare[4];
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1 0x00000000U
#define TIM_CHANNEL_2 0x00000004U
#define TIM_CHANNEL_3 0x00000008U
#define TIM_CHANNEL_4 0x0000000CU

void TestHalSetCompare(
    TIM_HandleTypeDef* timer,
    uint32_t channel,
    uint32_t compare);

#define __HAL_TIM_SET_COMPARE(timer, channel, compare) \
    TestHalSetCompare((timer), (channel), (compare))

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef HAL_UART_Receive_DMA(
    UART_HandleTypeDef* uart,
    uint8_t* data,
    uint16_t size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef* uart);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(
    UART_HandleTypeDef* uart,
    uint8_t* data,
    uint16_t size);
HAL_StatusTypeDef HAL_TIM_PWM_Start(
    TIM_HandleTypeDef* timer,
    uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(
    TIM_HandleTypeDef* timer,
    uint32_t channel);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(
    SPI_HandleTypeDef* spi,
    uint8_t* tx,
    uint8_t* rx,
    uint16_t length,
    uint32_t timeout_ms);
void HAL_GPIO_WritePin(
    GPIO_TypeDef* port,
    uint16_t pin,
    GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(
    GPIO_TypeDef* port,
    uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif /* TOOLS_TEST_SUPPORT_STM32F4XX_HAL_H */
