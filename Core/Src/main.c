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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "BNO085.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_ADC_CHANNELS 4
#define AVG_PER_CHANNEL 4

/*
 * Time Period for Interrupt:
 * - this htim16 runs at a 16mhz clock, and is currently set to be divided down by 1600
 * - that means the timer runs on a 10khz clock, and hits a timer interrupt after TIME_PERIOD - 1 ticks
 * - the TIME_PERIOD defines the frequency of CAN packet sends
 * - ex. TIME_PERIOD of 10000 means 1   interrupt/second
 * 		 TIME_PERIOD of 1000  means 10  interrupts/second
 * 		 TIME_PERIOD of 100   means 100 interrupts/second
 *
 * Use the correct time period for the sensors on the board (typically the fastest sensor)
 * Also note if the ioc changes the TIME_PERIOD will disappear so pls add this line back
 *
 * htim16.Init.Period = TIME_PERIOD - 1;
 *
 */
#define TIME_PERIOD 1000  
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef TxHeaderRotation;
uint8_t TxData[8] = {0};
uint32_t TxMailbox;
uint16_t ADC_DMA_BUFF[NUM_ADC_CHANNELS * AVG_PER_CHANNEL] = {0};
SH2_SensorEvent sensor_event = {0};
uint8_t can_interval = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
uint16_t ADC_DMA_AVG(uint16_t ADC_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief callback function for I2C receive
 * @param hi2c1: I2C handle
 * @retval None
 */
void I2C_Receive_Callback(I2C_HandleTypeDef *hi2c1)
{
  CAN_TxHeaderTypeDef *TxHeaderToUse = &TxHeader;
  if (can_interval % 100 == 0)
  {
    uint8_t sensor = BNO085_DecodeSensorEvent(TxData, &sensor_event);
    switch (sensor)
    {
    case SH2_ACCELEROMETER:
      TxHeaderToUse = &TxHeader;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      TxHeaderToUse = &TxHeader;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      TxHeaderToUse = &TxHeader;
      break;
    case SH2_ROTATION_VECTOR:
      TxHeaderToUse = &TxHeaderRotation;
      break;
    default:
      break;
    }

    HAL_CAN_AddTxMessage(&hcan, TxHeaderToUse, TxData, &TxMailbox);

    // Print Raw Report
//        for (int i = 0; i < sizeof(sensor_event); i++)
//        {
//          printf("%d: 0x%x\n", i, ((uint8_t *)&sensor_event)[i]);
//        }
  }
  can_interval++;
}

/*
 * @brief Averages an ADC channel over the DMA buffer, achieves better resolution
 * @param ADC_PIN - ADC channel pin
 * @retval uint16_t - averaged ADC DMA value on success, -1 if invalid pin
 */
uint16_t ADC_DMA_AVG(uint16_t ADC_Pin)
{
  int channel;
  int i;
  uint32_t adc_sum;

  switch (ADC_Pin)
  {
  case S1_Pin:
    channel = 0;
    break;
  case S2_Pin:
    channel = 1;
  case S3_Pin:
    channel = 2;
  case S4_Pin:
    channel = 3;
  default:
    channel = -1;
  }

  if (channel == -1)
  {
    return -1;
  }

  adc_sum = 0;
  for (i = 0; i < AVG_PER_CHANNEL; i++)
  {
    adc_sum += ADC_DMA_BUFF[channel + (i * NUM_ADC_CHANNELS)];
  }

  return adc_sum / AVG_PER_CHANNEL;
}

/*
 * @brief Packages ADC values into CAN packet
 * @param ADC_PIN - ADC channel pin
 * @retval bool - true on success
 */
bool ADC_CAN_Package(uint16_t ADC_Pin)
{
  uint16_t *value;
  switch (ADC_Pin)
  {
  case S1_Pin:
    value = (uint16_t *)&TxData[0];
    break;
  case S2_Pin:
    value = (uint16_t *)&TxData[2];
  case S3_Pin:
    value = (uint16_t *)&TxData[4];
  case S4_Pin:
    value = (uint16_t *)&TxData[6];
  default:
    value = NULL;
  }

  if (!value)
  {
    return 0;
  }

  *value = ADC_DMA_AVG(ADC_Pin);
  return 1;
}

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and send CAN packet
  if (htim == &htim16)
  {
     if (!ADC_CAN_Package(S1_Pin) || HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
     {
       Error_Handler();
     }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x111;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 6;

  TxHeaderRotation.IDE = CAN_ID_STD;
  TxHeaderRotation.StdId = 0x112;
  TxHeaderRotation.RTR = CAN_RTR_DATA;
  TxHeaderRotation.DLC = 8;
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
  // MX_DMA_Init();
  // MX_ADC_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  // MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)ADC_DMA_BUFF, NUM_ADC_CHANNELS * AVG_PER_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&htim16) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, I2C_Receive_Callback);

  // Create SHTP header
  SHTP_Header header = {0};
  header.length = 0x15;
  header.channel = SHTP_SENSOR_HUB_CHANNEL;
  header.sequence_number = 0x00;

  // Create SH2 Set Feature Command
  SHTP_Command start_accel = {0};
  start_accel.header = header;
  start_accel.header.sequence_number = 0x01;
  start_accel.report_id = BNO_COMMAND_SET_FEATURE_COMMAND;
  start_accel.feature_report_id = SH2_ACCELEROMETER;
  start_accel.report_interval = 0xEA60; // 60Hz

//  SHTP_Command start_gyro = {0};
//  start_gyro.header = header;
//  start_gyro.header.sequence_number = 0x02;
//  start_gyro.report_id = BNO_COMMAND_SET_FEATURE_COMMAND;
//  start_gyro.feature_report_id = SH2_GYROSCOPE_CALIBRATED;
//  start_gyro.report_interval = 0xEA60; // 60Hz
//
//  SHTP_Command start_mag = {0};
//  start_mag.header = header;
//  start_mag.header.sequence_number = 0x03;
//  start_mag.report_id = BNO_COMMAND_SET_FEATURE_COMMAND;
//  start_mag.feature_report_id = SH2_MAGNETIC_FIELD_CALIBRATED;
//  start_mag.report_interval = 0xEA60; // 60Hz
//
//  SHTP_Command start_rotation = {0};
//  start_rotation.header = header;
//  start_rotation.header.sequence_number = 0x04;
//  start_rotation.report_id = BNO_COMMAND_SET_FEATURE_COMMAND;
//  start_rotation.feature_report_id = SH2_ROTATION_VECTOR;
//  start_rotation.report_interval = 0xEA60; // 60Hz

  // Send command over I2C to BNO to start accelerometer
//  HAL_I2C_Master_Transmit(&hi2c1, BNO085_ADDRESS, (uint8_t *)&start_accel, sizeof(start_accel), 1000);
  // HAL_I2C_Master_Transmit(&hi2c1, BNO085_ADDRESS, (uint8_t *)&start_gyro, sizeof(start_gyro), 1000);
  // HAL_I2C_Master_Transmit(&hi2c1, BNO085_ADDRESS, (uint8_t *)&start_mag, sizeof(start_mag), 1000);
//  HAL_I2C_Master_Transmit(&hi2c1, BNO085_ADDRESS, (uint8_t *)&start_rotation, sizeof(start_rotation), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Get Input Report from BNO at 60Hz
//    HAL_I2C_Master_Receive_IT(&hi2c1, BNO085_ADDRESS, (uint8_t *)&sensor_event, sizeof(sensor_event));
//    HAL_Delay(16); // 60Hz = 16ms
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // set fifo assignment
  sFilterConfig.FilterIdHigh = 0;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // set filter scale
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00101D2D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  // Setting period manually and re-initing so I don't have to update every ioc change
  htim16.Init.Period = TIME_PERIOD - 1;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
