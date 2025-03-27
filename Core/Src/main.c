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
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_can.h"
#include "stm32f3xx_hal_conf.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO_08X_I2C.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t adc_pin;
  uint32_t sample_rate_ms;
  uint32_t last_sample_time;
} ADC_Sample_Config_t;

typedef struct {
  uint8_t sensor_id;
  uint32_t sample_rate_ms;
} IMU_Sample_Config_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#define NUM_ADC_CHANNELS 8
#define AVG_PER_CHANNEL 4
#define ADC_BIT_RESOLUTION 12

// Use fixed-point scaling for IMU data (3 decimal places of precision)
#define IMU_DATA_SCALE_FACTOR 1000.0f

#define FAST_SAMPLE_RATE_MS 10  // 100Hz
#define MED_SAMPLE_RATE_MS 20   // 50Hz
#define SLOW_SAMPLE_RATE_MS 200 // 5Hz

#define CAN_ID 0x400
#define CAN_ID_SECONDARY 0x401
#define CAN_FRAME_SIZE 8
#define CAN_MESSAGE_GROUP_SIZE 4
#define CAN_MESSAGE_SECONDARY_GROUP_SIZE 1

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
ADC + IMU CAN FRAME DEFINITION
MUX'd ADC Data Frame for 8 Channels (12 bits each) + IMU Accelerometer and
Gyroscope Data (16 bits each)

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

Mux 2:
  Bits 4-19: Accelerometer X (16 bits)
  Bits 20-35: Accelerometer Y (16 bits)
  Bits 36-51: Accelerometer Z (16 bits)

Mux 3:
  Bits 4-19: Gyroscope X (16 bits)
  Bits 20-35: Gyroscope Y (16 bits)
  Bits 36-51: Gyroscope Z (16 bits)
*/
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[CAN_FRAME_SIZE];
uint8_t MsgBuffers[CAN_MESSAGE_GROUP_SIZE][CAN_FRAME_SIZE] = {{0}};
bool msgPending[CAN_MESSAGE_GROUP_SIZE] = {false};

/*
 * Secondary CAN Frame for additional data
 * Currently needs to be used for the rotation vector
 *
 * Bits 0-15: Rotation Vector I (16 bits)
 * Bits 16-31: Rotation Vector J (16 bits)
 * Bits 32-47: Rotation Vector K (16 bits)
 * Bits 48-63: Rotation Vector R (16 bits)
 */
CAN_TxHeaderTypeDef TxHeaderSecondary;
uint8_t TxDataSecondary[CAN_FRAME_SIZE];
uint8_t MsgBuffersSecondary[CAN_MESSAGE_SECONDARY_GROUP_SIZE][CAN_FRAME_SIZE] =
    {{0}};
bool msgPendingSecondary[CAN_MESSAGE_SECONDARY_GROUP_SIZE] = {false};

uint32_t TxMailbox;
uint16_t last_adc_values[NUM_ADC_CHANNELS] = {0};
volatile uint8_t BNO_Ready = 0;

/*
 * ADC Channel Configuration
 * ADCs are named weirdly on the board, so I'm using 0-7 for clarity
 * ADC 0 = Channel 0, ADC 1 = Channel 1, etc.
 */
ADC_Sample_Config_t adc_configs[] = {
    {0, FAST_SAMPLE_RATE_MS, 0}, {1, FAST_SAMPLE_RATE_MS, 0},
    {2, MED_SAMPLE_RATE_MS, 0},  {3, MED_SAMPLE_RATE_MS, 0},
    {4, SLOW_SAMPLE_RATE_MS, 0}, {5, SLOW_SAMPLE_RATE_MS, 0},
    {6, SLOW_SAMPLE_RATE_MS, 0}, {7, SLOW_SAMPLE_RATE_MS, 0}};

/**
 * IMU Channel Configuration
 */
IMU_Sample_Config_t imu_configs[] = {{ACCELEROMETER, MED_SAMPLE_RATE_MS},
                                     {GYROSCOPE_CALIBRATED, MED_SAMPLE_RATE_MS},
                                     {ROTATION_VECTOR, MED_SAMPLE_RATE_MS}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/** INTERRUPT HANDLERS **/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == BNO_INT_Pin) {
    BNO_Ready = 1;
  }
}

/** ADC HANDLING **/

/**
 * @brief Averages an ADC channel over the DMA buffer, achieves better
 * resolution
 * @param ADC_PIN - ADC channel pin
 * @retval uint16_t - averaged ADC DMA value on success, -1 if invalid pin
 */
uint16_t ADC_DMA_AVG(uint16_t ADC_Pin) {
  assert_param(ADC_PIN_VALID(ADC_Pin));

  int channel;
  int i;
  uint32_t adc_sum;

  channel = ADC_Pin;

  adc_sum = 0;
  for (i = 0; i < AVG_PER_CHANNEL; i++) {
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
static void PackADCValue(uint8_t *buffer, uint16_t adc_value,
                         uint8_t channel_index) {
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
 * @brief Sends pending CAN messages
 * @retval bool - true if no pending messages
 */
/**
 * @brief Packages ADC values into CAN packet
 * @param ADC_PIN - ADC channel pin
 * @retval bool - true on success
 */
bool ADC_CAN_Package(uint16_t ADC_Pin) {
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
  MuxBuffer = (mux_val == 0) ? MsgBuffers[0] : MsgBuffers[1];

  // Clear only the mux nibble in first byte
  MuxBuffer[0] &= 0xF0;

  // Set first nibble to mux value
  MuxBuffer[0] |= mux_val;

  if (mux_val == 0) {
    // Set all ADC values for mux 0
    for (int i = 0; i < 5; i++) {
      PackADCValue(MuxBuffer, last_adc_values[i], i);
    }
    msgPending[0] = true;
  } else {
    // Set all ADC values for mux 1
    for (int i = 5; i < NUM_ADC_CHANNELS; i++) {
      PackADCValue(MuxBuffer, last_adc_values[i], i - 5);
    }
    msgPending[1] = true;
  }

  return true;
}

/** IMU HANDLING **/

/**
 * @brief Packages IMU values into CAN packet
 * @param buffer - Pointer to buffer (expected 8 bytes)
 * @param x - X-axis value
 * @param y - Y-axis value
 * @param z - Z-axis value
 * @retval bool - true on success
 */
void IMU_PackAccelGyro(uint8_t *buffer, float x, float y, float z) {
  assert_param(buffer != NULL);

  // Convert to 16-bit integer
  int16_t x_int = (int16_t)(x * IMU_DATA_SCALE_FACTOR);
  int16_t y_int = (int16_t)(y * IMU_DATA_SCALE_FACTOR);
  int16_t z_int = (int16_t)(z * IMU_DATA_SCALE_FACTOR);

  // Pack into buffer (little-endian)
  buffer[0] = x_int & 0xFF;
  buffer[1] = (x_int >> 8) & 0xFF;
  buffer[2] = y_int & 0xFF;
  buffer[3] = (y_int >> 8) & 0xFF;
  buffer[4] = z_int & 0xFF;
  buffer[5] = (z_int >> 8) & 0xFF;
}

/**
 * @brief Packages IMU values into CAN packet
 * @param buffer - Pointer to buffer (expected 8 bytes)
 * @param i - I-axis value
 * @param j - J-axis value
 * @param k - K-axis value
 * @param r - R-axis value
 * @retval bool - true on success
 */
void IMU_PackRotationVector(uint8_t *buffer, float i, float j, float k,
                            float r) {
  assert_param(buffer != NULL);

  // Convert to 16-bit integer
  int16_t i_int = (int16_t)(i * IMU_DATA_SCALE_FACTOR);
  int16_t j_int = (int16_t)(j * IMU_DATA_SCALE_FACTOR);
  int16_t k_int = (int16_t)(k * IMU_DATA_SCALE_FACTOR);
  int16_t r_int = (int16_t)(r * IMU_DATA_SCALE_FACTOR);

  // Pack into buffer (little-endian)
  buffer[0] = i_int & 0xFF;
  buffer[1] = (i_int >> 8) & 0xFF;
  buffer[2] = j_int & 0xFF;
  buffer[3] = (j_int >> 8) & 0xFF;
  buffer[4] = k_int & 0xFF;
  buffer[5] = (k_int >> 8) & 0xFF;
  buffer[6] = r_int & 0xFF;
  buffer[7] = (r_int >> 8) & 0xFF;
}

/**
 * @brief Packages IMU values into CAN packet
 * @param sensor_id - Sensor ID (ACCELEROMETER, GYROSCOPE_CALIBRATED, etc.)
 * @param sensor_data - Pointer to sensor data
 * @retval bool - true on success
 */
bool IMU_CAN_Package(uint8_t sensor_id, BNO_SensorValue_t *sensor_data) {
  uint8_t *MuxBuffer;
  uint8_t mux_val;

  switch (sensor_id) {
  case ACCELEROMETER:
    mux_val = 2;
    MuxBuffer = MsgBuffers[2];
    IMU_PackAccelGyro(MuxBuffer, sensor_data->SenVal.Accelerometer.X,
                      sensor_data->SenVal.Accelerometer.Y,
                      sensor_data->SenVal.Accelerometer.Z);
    // Shift values down to make room for mux nibble
    *MuxBuffer >>= 4;
    *MuxBuffer |= mux_val;
    msgPending[2] = true;
    break;
  case GYROSCOPE_CALIBRATED:
    mux_val = 3;
    MuxBuffer = MsgBuffers[3];
    IMU_PackAccelGyro(MuxBuffer, sensor_data->SenVal.Gyroscope.X,
                      sensor_data->SenVal.Gyroscope.Y,
                      sensor_data->SenVal.Gyroscope.Z);
    // Shift values down to make room for mux nibble
    *MuxBuffer >>= 4;
    *MuxBuffer |= mux_val;
    msgPending[3] = true;
    break;
  case ROTATION_VECTOR:
    MuxBuffer = MsgBuffersSecondary[0];
    IMU_PackRotationVector(MuxBuffer, sensor_data->SenVal.RotationVector.I,
                           sensor_data->SenVal.RotationVector.J,
                           sensor_data->SenVal.RotationVector.K,
                           sensor_data->SenVal.RotationVector.Real);
    break;
  default:
    return false;
  }

  return true;
}

/**
 * @brief Sets IMU reports for all sensors
 */
void setIMUReports(void) {
  for (unsigned int i = 0; i < sizeof(imu_configs) / sizeof(imu_configs[0]);
       i++) {
    if (BNO_setFeature(imu_configs[i].sensor_id,
                       imu_configs[i].sample_rate_ms * 1000, 0) == HAL_OK) {
      printf("Set report for sensor %d at %d Hz\r\n", imu_configs[i].sensor_id,
             1000 / imu_configs[i].sample_rate_ms);
    } else {
      printf("Failed to set report for sensor %d at %d Hz\r\n",
             imu_configs[i].sensor_id, 1000 / imu_configs[i].sample_rate_ms);
    }
  }
}

/** CAN HANDLING **/

/**
 * @brief Sends pending CAN messages
 * @retval bool - true if no pending messages
 */
bool SendPendingCANMessages(void) {
  bool status;
  // Primary CAN ID
  for (int i = 0; i < CAN_MESSAGE_GROUP_SIZE; i++) {
    if (msgPending[i]) {
      memcpy(TxData, MsgBuffers[i], 8);
      if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) ==
          HAL_OK) {
        msgPending[i] = false;
      } else {
        status = false;
      }
    }
  }

  // Secondary CAN ID
  for (int i = 0; i < CAN_MESSAGE_SECONDARY_GROUP_SIZE; i++) {
    if (msgPendingSecondary[i]) {
      memcpy(TxDataSecondary, MsgBuffersSecondary[i], 8);
      if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderSecondary, TxDataSecondary,
                               &TxMailbox) == HAL_OK) {
        msgPendingSecondary[i] = false;
      } else {
        status = false;
      }
    }
  }

  return status;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  // Initialize CAN header
  TxHeader.StdId = CAN_ID;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;

  // Initialize secondary CAN header
  TxHeaderSecondary.StdId = CAN_ID_SECONDARY;
  TxHeaderSecondary.IDE = CAN_ID_STD;
  TxHeaderSecondary.RTR = CAN_RTR_DATA;
  TxHeaderSecondary.DLC = 8;
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    printf("Failed to start CAN peripheral\r\n");
    Error_Handler();
  }

  // Start ADC DMA
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_DMA_BUFF,
                        NUM_ADC_CHANNELS * AVG_PER_CHANNEL) != HAL_OK) {
    printf("Failed to start ADC DMA\r\n");
    Error_Handler();
  }

  // Initialize BNO085
  if (BNO_Init() != HAL_OK) {
    printf("Failed to initialize BNO085\r\n");
    Error_Handler();
  }

  // Set feature reports for IMU
  BNO_setHighAccuracyMode();
  setIMUReports();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_time = HAL_GetTick();

    // Check and sample each ADC channel based on its configured rate
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
      if ((current_time - adc_configs[i].last_sample_time) >=
          adc_configs[i].sample_rate_ms) {
        // Sample this channel
        if (ADC_CAN_Package(adc_configs[i].adc_pin)) {
          adc_configs[i].last_sample_time = current_time;
        }
      }
    }

    if (BNO_Ready) {
      // Check if reset occurred and send new reports
      if (isResetOccurred()) {
        printf("BNO085 reset occurred!! Resending feature reports...\r\n");
        setIMUReports();
      }

      // Handle IMU data
      if (BNO_dataAvailable() == HAL_OK) {
        uint8_t sensorID = BNO_getSensorEventID();
        printf("Received data from sensor %d\r\n", sensorID);
        IMU_CAN_Package(sensorID, &sensorData);
      }
    }

    // Send pending CAN messages
    SendPendingCANMessages();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection =
      RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PCLK2_DIV2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
    printf("Error occurred!\r\n");
    HAL_Delay(1000);
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  printf("ASSERT FAILED: %s:%d\r\n", file, line);
  Error_Handler();
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
