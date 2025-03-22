/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t adc_pin;
  uint32_t sample_rate_ms;
  uint32_t last_sample_time;
} ADC_Sample_Config_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_ADC_CHANNELS 8
#define AVG_PER_CHANNEL 4
#define ADC_BIT_RESOLUTION 12

#define FAST_SAMPLE_RATE_MS 10   // 100Hz
#define MED_SAMPLE_RATE_MS 100   // 10Hz
#define SLOW_SAMPLE_RATE_MS 1000 // 1Hz

#define CAN_ID 0x400
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_PIN_VALID(pin) ((pin) >= 0 && (pin) < NUM_ADC_CHANNELS)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static __attribute__((
    aligned(4))) uint16_t ADC_DMA_BUFF[NUM_ADC_CHANNELS * AVG_PER_CHANNEL];

/*
ADC CAN FRAME DEFINITION
MUX'd ADC Data Frame for 8 Channels (12 bits each)

Bits 0-3: Mux Nibble

Mux 0:
  Bits 4-15: ADC Channel 1 (12 bits)
  Bits 16-27: ADC Channel 2 (12 bits)
  Bits 28-39: ADC Channel 3 (12 bits)
  Bits 40-51: ADC Channel 4 (12 bits)
  Bits 52-63: ADC Channel 5 (12 bits)

Mux 1:
  Bits 4-15: ADC Channel 6 (12 bits)
  Bits 16-27: ADC Channel 7 (12 bits)
  Bits 28-39: ADC Channel 8 (12 bits)
*/
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint8_t AdcMux0Buffer[8];
uint8_t AdcMux1Buffer[8];
bool mux0_pending = false;
bool mux1_pending = false;
uint32_t TxMailbox;

uint16_t last_adc_values[NUM_ADC_CHANNELS] = {0};

/*
 * ADC Channel Configuration
 * ADCs are named weirdly on the board, so I'm using 0-7 for clarity
 * ADC 0 = Channel 0, ADC 1 = Channel 1, etc.
 */
ADC_Sample_Config_t adc_configs[] = {
    {0, FAST_SAMPLE_RATE_MS, 0},
    {1, FAST_SAMPLE_RATE_MS, 0},
    {2, MED_SAMPLE_RATE_MS, 0},
    {3, MED_SAMPLE_RATE_MS, 0},
    {4, SLOW_SAMPLE_RATE_MS, 0},
    {5, SLOW_SAMPLE_RATE_MS, 0},
    {6, SLOW_SAMPLE_RATE_MS, 0},
    {7, SLOW_SAMPLE_RATE_MS, 0}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Averages an ADC channel over the DMA buffer, achieves better
 * resolution
 * @param ADC_PIN - ADC channel pin
 * @retval uint16_t - averaged ADC DMA value on success, -1 if invalid pin
 */
uint16_t ADC_DMA_AVG(uint16_t ADC_Pin)
{
  assert_param(ADC_PIN_VALID(ADC_Pin));

  int channel;
  int i;
  uint32_t adc_sum;

  channel = ADC_Pin;

  adc_sum = 0;
  for (i = 0; i < AVG_PER_CHANNEL; i++)
  {
    adc_sum += ADC_DMA_BUFF[channel + (i * NUM_ADC_CHANNELS)];
  }

  return adc_sum / AVG_PER_CHANNEL;
}

/**
 * @brief Helper function to pack a 12-bit ADC value into the CAN buffer
 * @param buffer - Pointer to buffer (Mux0 or Mux1)
 * @param adc_value - 12-bit ADC value to pack
 * @param channel_index - Channel index (0-7, relative to start of frame)
 */
static void PackADCValue(uint8_t *buffer, uint16_t adc_value, uint8_t channel_index)
{
  // Only use 12 bits of ADC value
  adc_value &= 0x0FFF;

  // Calculate bit position (4 bits for mux nibble + 12 bits per channel)
  uint8_t bit_position = 4 + (channel_index * ADC_BIT_RESOLUTION);
  uint8_t byte_index = bit_position / 8;
  uint8_t bit_offset = bit_position % 8;
  uint8_t bits_in_first_byte = 8 - bit_offset;

  // Clear bits and set new value in first byte
  buffer[byte_index] &= ~(0xFF << bit_offset);
  buffer[byte_index] |= (adc_value << bit_offset);

  // Clear bits and set new value in second byte
  buffer[byte_index + 1] &= ~(0xFF >> bits_in_first_byte);
  buffer[byte_index + 1] |= (adc_value >> bits_in_first_byte);
}

/**
 * @brief Packages ADC values into CAN packet
 * @param ADC_PIN - ADC channel pin
 * @retval bool - true on success
 */
bool ADC_CAN_Package(uint16_t ADC_Pin)
{
  assert_param(ADC_PIN_VALID(ADC_Pin));

  uint16_t adc_val;
  uint8_t mux_val;
  uint8_t channel;
  uint8_t *MuxBuffer;

  channel = ADC_Pin;

  // Sample channel and store value
  adc_val = ADC_DMA_AVG(ADC_Pin);
  last_adc_values[channel] = adc_val;

  // Set mux value and select appropriate buffer
  mux_val = (channel < 6) ? 0 : 1;
  MuxBuffer = (mux_val == 0) ? AdcMux0Buffer : AdcMux1Buffer;

  // Clear only the mux nibble in first byte
  MuxBuffer[0] &= 0xF0;

  // Set first nibble to mux value
  MuxBuffer[0] |= mux_val;

  if (mux_val == 0)
  {
    // Set all ADC values for mux 0
    for (int i = 0; i < 5; i++)
    {
      PackADCValue(MuxBuffer, last_adc_values[i], i);
    }
    mux0_pending = true;
  }
  else
  {
    // Set all ADC values for mux 1
    for (int i = 5; i < NUM_ADC_CHANNELS; i++)
    {
      PackADCValue(MuxBuffer, last_adc_values[i], i - 5);
    }
    mux1_pending = true;
  }

  return true;
}

/**
 * @brief Sends pending CAN messages
 * @retval bool - true if no pending messages
 */
bool SendPendingCANMessages(void)
{
  if (mux0_pending)
  {
    // Copy mux 0 buffer to TX buffer and send
    memcpy(TxData, AdcMux0Buffer, 8);
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK)
    {
      mux0_pending = false;
    }
  }

  if (mux1_pending)
  {
    memcpy(TxData, AdcMux1Buffer, 8);
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) ==
        HAL_OK)
    {
      mux1_pending = false;
    }
  }

  return !(mux0_pending || mux1_pending);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Initialize CAN header
  TxHeader.StdId = CAN_ID;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_DMA_BUFF,
                        NUM_ADC_CHANNELS * AVG_PER_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_time = HAL_GetTick();

    // Check and sample each ADC channel based on its configured rate
    for (int i = 0; i < NUM_ADC_CHANNELS; i++)
    {
      if ((current_time - adc_configs[i].last_sample_time) >=
          adc_configs[i].sample_rate_ms)
      {
        // Sample this channel
        if (ADC_CAN_Package(adc_configs[i].adc_pin))
        {
          adc_configs[i].last_sample_time = current_time;
        }
      }
    }

    // Send pending CAN messages
    SendPendingCANMessages();

    // Small delay to prevent overwhelming the CPU
    HAL_Delay(1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_ADC1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PCLK2_DIV2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef USE_FULL_ASSERT
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
  Error_Handler();
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
