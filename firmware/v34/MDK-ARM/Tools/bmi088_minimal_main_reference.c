/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile HAL_StatusTypeDef bmi088_test_acc_activation_status = HAL_ERROR;
volatile HAL_StatusTypeDef bmi088_test_acc_status = HAL_ERROR;
volatile HAL_StatusTypeDef bmi088_test_gyro_status = HAL_ERROR;
volatile uint8_t bmi088_test_acc_activation_rx[3] = {0U, 0U, 0U};
volatile uint8_t bmi088_test_acc_rx[3] = {0U, 0U, 0U};
volatile uint8_t bmi088_test_gyro_rx[2] = {0U, 0U};
volatile uint8_t bmi088_test_acc_chip_id = 0U;
volatile uint8_t bmi088_test_gyro_chip_id = 0U;
volatile uint8_t bmi088_test_pass = 0U;
volatile uint8_t bmi088_test_step = 0U;
volatile uint8_t bmi088_test_done = 0U;
volatile HAL_StatusTypeDef bmi088_test_init_status = HAL_ERROR;
volatile HAL_StatusTypeDef bmi088_test_sample_status = HAL_ERROR;
volatile uint8_t bmi088_test_init_pass = 0U;
volatile uint8_t bmi088_test_acc_data[6] = {0U};
volatile uint8_t bmi088_test_gyro_data[6] = {0U};
volatile uint8_t bmi088_test_temperature_data[2] = {0U};
volatile int16_t bmi088_test_acc_x = 0;
volatile int16_t bmi088_test_acc_y = 0;
volatile int16_t bmi088_test_acc_z = 0;
volatile int16_t bmi088_test_gyro_x = 0;
volatile int16_t bmi088_test_gyro_y = 0;
volatile int16_t bmi088_test_gyro_z = 0;
volatile int16_t bmi088_test_temperature_raw = 0;
volatile float bmi088_test_temperature_deg_c = 0.0f;
volatile uint32_t bmi088_test_sample_count = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef BMI088_AccTransfer(uint8_t *tx, uint8_t *rx, uint16_t length);
static HAL_StatusTypeDef BMI088_GyroTransfer(uint8_t *tx, uint8_t *rx, uint16_t length);
static HAL_StatusTypeDef BMI088_WriteAccRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef BMI088_WriteGyroRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef BMI088_ReadAccRegisters(uint8_t reg, uint8_t *data, uint8_t length);
static HAL_StatusTypeDef BMI088_ReadGyroRegisters(uint8_t reg, uint8_t *data, uint8_t length);
static HAL_StatusTypeDef BMI088_MinimalInit(void);
static HAL_StatusTypeDef BMI088_ReadSample(void);
static int16_t BMI088_DecodeLittleEndian(const uint8_t *data);
static int16_t BMI088_DecodeTemperature(const uint8_t *data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef BMI088_AccTransfer(uint8_t *tx, uint8_t *rx, uint16_t length)
{
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, length, 100U);
  HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
  return status;
}

static HAL_StatusTypeDef BMI088_GyroTransfer(uint8_t *tx, uint8_t *rx, uint16_t length)
{
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, length, 100U);
  HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
  return status;
}

static HAL_StatusTypeDef BMI088_WriteAccRegister(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {reg, value};
  uint8_t rx[2] = {0U, 0U};
  return BMI088_AccTransfer(tx, rx, 2U);
}

static HAL_StatusTypeDef BMI088_WriteGyroRegister(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {reg, value};
  uint8_t rx[2] = {0U, 0U};
  return BMI088_GyroTransfer(tx, rx, 2U);
}

static HAL_StatusTypeDef BMI088_ReadAccRegisters(uint8_t reg, uint8_t *data, uint8_t length)
{
  uint8_t tx[8] = {0U};
  uint8_t rx[8] = {0U};
  HAL_StatusTypeDef status;
  uint8_t index;

  if ((data == NULL) || (length == 0U) || (length > 6U)) {
    return HAL_ERROR;
  }

  tx[0] = reg | 0x80U;
  status = BMI088_AccTransfer(tx, rx, (uint16_t)(length + 2U));
  if (status != HAL_OK) {
    return status;
  }
  for (index = 0U; index < length; ++index) {
    data[index] = rx[index + 2U];
  }
  return HAL_OK;
}

static HAL_StatusTypeDef BMI088_ReadGyroRegisters(uint8_t reg, uint8_t *data, uint8_t length)
{
  uint8_t tx[7] = {0U};
  uint8_t rx[7] = {0U};
  HAL_StatusTypeDef status;
  uint8_t index;

  if ((data == NULL) || (length == 0U) || (length > 6U)) {
    return HAL_ERROR;
  }

  tx[0] = reg | 0x80U;
  status = BMI088_GyroTransfer(tx, rx, (uint16_t)(length + 1U));
  if (status != HAL_OK) {
    return status;
  }
  for (index = 0U; index < length; ++index) {
    data[index] = rx[index + 1U];
  }
  return HAL_OK;
}

static HAL_StatusTypeDef BMI088_MinimalInit(void)
{
  uint8_t acc_tx[3] = {0x80U, 0x00U, 0x00U};
  uint8_t gyro_tx[2] = {0x80U, 0x00U};
  uint8_t dummy = 0U;
  uint8_t acc_id = 0U;
  uint8_t gyro_id = 0U;
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(10U);

  /* First ACC transaction switches the sensor interface to SPI. */
  bmi088_test_step = 1U;
  bmi088_test_acc_activation_status = BMI088_AccTransfer(
      acc_tx, (uint8_t *)bmi088_test_acc_activation_rx, 3U);
  if (bmi088_test_acc_activation_status != HAL_OK) {
    return bmi088_test_acc_activation_status;
  }
  HAL_Delay(1U);

  /* ACC reads have one dummy byte, so CHIP_ID is RX[2]. */
  bmi088_test_step = 2U;
  bmi088_test_acc_status = BMI088_AccTransfer(
      acc_tx, (uint8_t *)bmi088_test_acc_rx, 3U);
  if (bmi088_test_acc_status != HAL_OK) {
    return bmi088_test_acc_status;
  }

  /* GYRO reads have no extra dummy byte, so CHIP_ID is RX[1]. */
  bmi088_test_step = 3U;
  bmi088_test_gyro_status = BMI088_GyroTransfer(
      gyro_tx, (uint8_t *)bmi088_test_gyro_rx, 2U);
  if (bmi088_test_gyro_status != HAL_OK) {
    return bmi088_test_gyro_status;
  }

  bmi088_test_acc_chip_id = bmi088_test_acc_rx[2];
  bmi088_test_gyro_chip_id = bmi088_test_gyro_rx[1];
  bmi088_test_pass = (bmi088_test_acc_chip_id == 0x1EU) &&
                     (bmi088_test_gyro_chip_id == 0x0FU);
  bmi088_test_step = 4U;
  if (bmi088_test_pass == 0U) {
    return HAL_ERROR;
  }

  bmi088_test_step = 5U;
  status = BMI088_WriteAccRegister(0x7EU, 0xB6U);
  if (status != HAL_OK) {
    return status;
  }
  HAL_Delay(1U);

  bmi088_test_step = 6U;
  status = BMI088_ReadAccRegisters(0x00U, &dummy, 1U);
  if (status != HAL_OK) {
    return status;
  }

  bmi088_test_step = 7U;
  status = BMI088_WriteGyroRegister(0x14U, 0xB6U);
  if (status != HAL_OK) {
    return status;
  }
  HAL_Delay(30U);

  bmi088_test_step = 8U;
  status = BMI088_ReadAccRegisters(0x00U, &acc_id, 1U);
  if (status != HAL_OK) {
    return status;
  }
  status = BMI088_ReadGyroRegisters(0x00U, &gyro_id, 1U);
  if (status != HAL_OK) {
    return status;
  }
  bmi088_test_acc_chip_id = acc_id;
  bmi088_test_gyro_chip_id = gyro_id;
  bmi088_test_pass = (bmi088_test_acc_chip_id == 0x1EU) &&
                     (bmi088_test_gyro_chip_id == 0x0FU);
  if (bmi088_test_pass == 0U) {
    return HAL_ERROR;
  }

  bmi088_test_step = 9U;
  status = BMI088_WriteAccRegister(0x7CU, 0x00U);
  if (status != HAL_OK) {
    return status;
  }
  HAL_Delay(5U);

  bmi088_test_step = 10U;
  status = BMI088_WriteAccRegister(0x7DU, 0x04U);
  if (status != HAL_OK) {
    return status;
  }
  HAL_Delay(5U);

  bmi088_test_step = 11U;
  status = BMI088_WriteAccRegister(0x40U, 0xA9U);
  if (status != HAL_OK) {
    return status;
  }
  HAL_Delay(40U);

  bmi088_test_step = 12U;
  status = BMI088_WriteAccRegister(0x41U, 0x01U);
  if (status != HAL_OK) {
    return status;
  }
  HAL_Delay(1U);

  bmi088_test_step = 13U;
  status = BMI088_WriteGyroRegister(0x10U, 0x04U);
  if (status != HAL_OK) {
    return status;
  }
  HAL_Delay(1U);

  bmi088_test_step = 14U;
  status = BMI088_WriteGyroRegister(0x0FU, 0x00U);
  if (status != HAL_OK) {
    return status;
  }
  HAL_Delay(10U);

  bmi088_test_step = 15U;
  return HAL_OK;
}

static int16_t BMI088_DecodeLittleEndian(const uint8_t *data)
{
  uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
  return (int16_t)value;
}

static int16_t BMI088_DecodeTemperature(const uint8_t *data)
{
  int16_t value = (int16_t)(((uint16_t)data[0] << 3U) |
                            ((uint16_t)data[1] >> 5U));
  if (value > 1023) {
    value = (int16_t)(value - 2048);
  }
  return value;
}

static HAL_StatusTypeDef BMI088_ReadSample(void)
{
  uint8_t acc_data[6] = {0U};
  uint8_t gyro_data[6] = {0U};
  uint8_t temperature_data[2] = {0U};
  HAL_StatusTypeDef status;
  uint8_t index;

  status = BMI088_ReadAccRegisters(0x12U, acc_data, 6U);
  if (status != HAL_OK) {
    return status;
  }
  status = BMI088_ReadGyroRegisters(0x02U, gyro_data, 6U);
  if (status != HAL_OK) {
    return status;
  }
  status = BMI088_ReadAccRegisters(0x22U, temperature_data, 2U);
  if (status != HAL_OK) {
    return status;
  }

  for (index = 0U; index < 6U; ++index) {
    bmi088_test_acc_data[index] = acc_data[index];
    bmi088_test_gyro_data[index] = gyro_data[index];
  }
  bmi088_test_temperature_data[0] = temperature_data[0];
  bmi088_test_temperature_data[1] = temperature_data[1];

  bmi088_test_acc_x = BMI088_DecodeLittleEndian(&acc_data[0]);
  bmi088_test_acc_y = BMI088_DecodeLittleEndian(&acc_data[2]);
  bmi088_test_acc_z = BMI088_DecodeLittleEndian(&acc_data[4]);
  bmi088_test_gyro_x = BMI088_DecodeLittleEndian(&gyro_data[0]);
  bmi088_test_gyro_y = BMI088_DecodeLittleEndian(&gyro_data[2]);
  bmi088_test_gyro_z = BMI088_DecodeLittleEndian(&gyro_data[4]);
  bmi088_test_temperature_raw = BMI088_DecodeTemperature(temperature_data);
  bmi088_test_temperature_deg_c =
      ((float)bmi088_test_temperature_raw * 0.125f) + 23.0f;
  ++bmi088_test_sample_count;
  return HAL_OK;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  bmi088_test_init_status = BMI088_MinimalInit();
  bmi088_test_init_pass = (bmi088_test_init_status == HAL_OK) ? 1U : 0U;
  bmi088_test_done = 1U;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (bmi088_test_init_pass != 0U)
    {
      bmi088_test_sample_status = BMI088_ReadSample();
    }
    HAL_Delay(10U);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and the line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
