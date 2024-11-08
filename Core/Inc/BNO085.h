/**
 * @file BNO085.h
 * @brief IMU driver for BNO085
 * @date 2024-03-21
 * @author Samrut Gadde
 * @version 1.0
 */

#ifndef INC_BNO085_H_
#define INC_BNO085_H_

#include "stm32f0xx.h"
#include <stdbool.h>

#define BNO085_ADDRESS 0x4A << 1

// Define this to print debug messages
#define DEBUG

/* SHTP CHANNELS */
#define SHTP_COMMAND_CHANNEL 0x00
#define SHTP_EXECUTABLE_CHANNEL 0x01
#define SHTP_SENSOR_HUB_CHANNEL 0x02
#define SHTP_INPUT_REPORT_CHANNEL 0x03
#define SHTP_WAKE_REPORT_CHANNEL 0x04
#define SHTP_GYRO_CHANNEL 0x05

/* BNO08X COMMANDS (REQUIRES SHTP_SENSOR_HUB_CHANNEL) */
#define BNO_COMMAND_GET_FEATURE_REQUEST 0xFE
#define BNO_COMMAND_SET_FEATURE_COMMAND 0xFD
#define BNO_COMMAND_GET_FEATURE_RESPONSE 0xFC
#define BNO_COMMAND_PRODUCT_ID_REQUEST 0xF9
#define BNO_COMMAND_PRODUCT_ID_RESPONSE 0xF8
#define BNO_COMMAND_FRS_WRITE_REQUEST 0xF7
#define BNO_COMMAND_FRS_WRITE_DATA 0xF6
#define BNO_COMMAND_FRS_WRITE_RESPONSE 0xF5
#define BNO_COMMAND_FRS_READ_REQUEST 0xF4
#define BNO_COMMAND_FRS_READ_RESPONSE 0xF3
#define BNO_COMMAND_REQUEST 0xF2
#define BNO_COMMAND_RESPONSE 0xF1

/* SH-2 SENSOR REPORT IDS */
enum SH2_SensorReportID
{
  SH2_RAW_ACCELEROMETER = 0x14,
  SH2_ACCELEROMETER = 0x01,
  SH2_LINEAR_ACCELERATION = 0x04,
  SH2_GRAVITY = 0x06,
  SH2_RAW_GYROSCOPE = 0x15,
  SH2_GYROSCOPE_CALIBRATED = 0x02,
  SH2_GYROSCOPE_UNCALIBRATED = 0x07,
  SH2_RAW_MAGNETOMETER = 0x16,
  SH2_MAGNETIC_FIELD_CALIBRATED = 0x03,
  SH2_MAGNETIC_FIELD_UNCALIBRATED = 0x0F,
  SH2_ROTATION_VECTOR = 0x05,
  SH2_GEOMAGNETIC_ROTATION_VECTOR = 0x09,
  SH2_PRESSURE = 0x0A,
  SH2_ARVR_STABILIZED_RV = 0x29,
  SH2_GYRO_INTEGRATED_RV = 0x2A,
  SH2_MOTION_REQUEST = 0x2B
};
// There are more but are unnecessary for this project

/* SHTP HEADER */
typedef struct __attribute__((packed)) SHTP_Header
{
  uint16_t length;
  uint8_t channel;
  uint8_t sequence_number;
} SHTP_Header;

typedef struct __attribute__((packed)) SHTP_Command
{
  SHTP_Header header;
  uint8_t report_id;
  uint8_t feature_report_id;
  uint8_t feature_flags;
  uint16_t change_sensitivity;
  uint32_t report_interval;
  uint32_t batch_interval;
  uint32_t sensor_specific_config;
} SHTP_Command;

typedef struct __attribute__((packed)) SHTP_CommandRequest
{
  // always BNO_COMMAND_REQUEST
  uint8_t report_id;
  uint8_t sequence_number;
  uint8_t command;
  uint8_t command_parameters[9];
} SHTP_CommandRequest;

typedef struct __attribute__((packed)) SHTP_CommandResponse
{
  // always BNO_COMMAND_RESPONSE
  uint8_t report_id;
  uint8_t sequence_number;
  uint8_t command;
  uint8_t command_sequence_number;
  uint8_t response_sequence_number;
  uint8_t command_parameters[11];
} SHTP_CommandResponse;

typedef struct __attribute__((packed)) SH2_Timestamp
{
  uint8_t report_id;
  uint32_t base_delta;
} SH2_Timestamp;

typedef struct __attribute__((packed)) SH2_Accelerometer
{
  int16_t x;
  int16_t y;
  int16_t z;
} SH2_Accelerometer;

typedef struct __attribute__((packed)) SH2_Gyroscope_Calibrated
{
  int16_t x;
  int16_t y;
  int16_t z;
} SH2_Gyroscope_Calibrated;

typedef struct __attribute__((packed)) SH2_Gyroscope_Uncalibrated
{
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t x_bias;
  int16_t y_bias;
  int16_t z_bias;
} SH2_Gyroscope_Uncalibrated;

typedef struct __attribute__((packed)) SH2_Magnetometer_Calibrated
{
  int16_t x;
  int16_t y;
  int16_t z;
} SH2_Magnetometer_Calibrated;

typedef struct __attribute__((packed)) SH2_Rotation_Vector
{
  int16_t i;
  int16_t j;
  int16_t k;
  int16_t real;
  int16_t accuracy;
} SH2_Rotation_Vector;

typedef struct __attribute__((packed)) SH2_SensorEvent
{
  SHTP_Header header;
  SH2_Timestamp timestamp;
  uint8_t report_id;
  uint8_t sequence_number;
  uint8_t status;
  uint8_t delay;
  union SH2_SensorData
  {
    SH2_Accelerometer accelerometer;
    SH2_Gyroscope_Calibrated gyroscope_calibrated;
    // SH2_Gyroscope_Uncalibrated gyroscope_uncalibrated;
    SH2_Magnetometer_Calibrated magnetometer_calibrated;
    SH2_Rotation_Vector rotation_vector;
  } data;
} SH2_SensorEvent;

enum SH2_SensorReportID BNO085_DecodeSensorEvent(uint8_t *data, SH2_SensorEvent *event);

#endif /* INC_BNO085_H_ */
