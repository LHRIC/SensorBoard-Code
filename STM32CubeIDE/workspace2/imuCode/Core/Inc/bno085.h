#ifndef _BNO085_H
#define _BNO085_H

#include "main.h"
#include "stm32f4xx_hal.h" // Include the correct header for STM32F4

//////////////////////////////////////////////////////////////////////////

/**
 * @brief Definition for I2C
 */
#define BNO085_ADDRESS (0x4A << 1) // I2C address for BNO085

#define BNO085_PS0_WAKE_PIN LL_GPIO_PIN_8
#define BNO085_PS0_WAKE_PORT GPIOA
#define BNO085_PS0_WAKE_CLK LL_AHB1_GRP1_PERIPH_GPIOA

#define BNO085_RST_PIN LL_GPIO_PIN_9
#define BNO085_RST_PORT GPIOC
#define BNO085_RST_CLK LL_AHB1_GRP1_PERIPH_GPIOC

#define BNO085_INT_PIN LL_GPIO_PIN_8
#define BNO085_INT_PORT GPIOC
#define BNO085_INT_CLK LL_AHB1_GRP1_PERIPH_GPIOC

//////////////////////////////////////////////////////////////////////////

#define WAKE_HIGH() LL_GPIO_SetOutputPin(BNO085_PS0_WAKE_PORT, BNO085_PS0_WAKE_PIN)
#define WAKE_LOW() LL_GPIO_ResetOutputPin(BNO085_PS0_WAKE_PORT, BNO085_PS0_WAKE_PIN)

#define RESET_HIGH() LL_GPIO_SetOutputPin(BNO085_RST_PORT, BNO085_RST_PIN)
#define RESET_LOW() LL_GPIO_ResetOutputPin(BNO085_RST_PORT, BNO085_RST_PIN)

//////////////////////////////////////////////////////////////////////////

// Registers
enum Registers {
    CHANNEL_COMMAND = 0,
    CHANNEL_EXECUTABLE = 1,
    CHANNEL_CONTROL = 2,
    CHANNEL_REPORTS = 3,
    CHANNEL_WAKE_REPORTS = 4,
    CHANNEL_GYRO = 5
};

// All the ways we can configure or talk to the BNO085, figure 34, page 36 reference manual
// These are used for low-level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

// All the different sensors and features we can get reports from
// These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

// Record IDs from figure 29, page 29 reference manual
// These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

// Command IDs from section 6.4, page 42
// These are used to calibrate, initialize, set orientation, tare etc. the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 // Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 // This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc.)

void BNO085_GPIO_I2C_Initialization(void);
int BNO085_Initialization(I2C_HandleTypeDef *hi2c);

int BNO085_dataAvailable(I2C_HandleTypeDef *hi2c);
void BNO085_parseCommandReport(void);
void BNO085_parseInputReport(void);

float BNO085_getQuatI();
float BNO085_getQuatJ();
float BNO085_getQuatK();
float BNO085_getQuatReal();
float BNO085_getQuatRadianAccuracy();
uint8_t BNO085_getQuatAccuracy();
float BNO085_getAccelX();
float BNO085_getAccelY();
float BNO085_getAccelZ();
uint8_t BNO085_getAccelAccuracy();
float BNO085_getLinAccelX();
float BNO085_getLinAccelY();
float BNO085_getLinAccelZ();
uint8_t BNO085_getLinAccelAccuracy();
float BNO085_getGyroX();
float BNO085_getGyroY();
float BNO085_getGyroZ();
uint8_t BNO085_getGyroAccuracy();
float BNO085_getMagX();
float BNO085_getMagY();
float BNO085_getMagZ();
uint8_t BNO085_getMagAccuracy();
uint16_t BNO085_getStepCount();
uint8_t BNO085_getStabilityClassifier();
uint8_t BNO085_getActivityClassifier();
uint32_t BNO085_getTimeStamp();
int16_t BNO085_getQ1(uint16_t recordID);
int16_t BNO085_getQ2(uint16_t recordID);
int16_t BNO085_getQ3(I2C_HandleTypeDef *hi2c, uint16_t recordID);
float BNO085_getResolution(I2C_HandleTypeDef *hi2c, uint16_t recordID);
float BNO085_getRange(I2C_HandleTypeDef *hi2c, uint16_t recordID);
int BNO085_sendPacket(I2C_HandleTypeDef *hi2c, uint8_t channelNumber, uint8_t dataLength);
uint32_t BNO085_readFRSword(I2C_HandleTypeDef *hi2c, uint16_t recordID, uint8_t wordNumber);
void BNO085_frsReadRequest(I2C_HandleTypeDef *hi2c, uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
int BNO085_readFRSdata(I2C_HandleTypeDef *hi2c, uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);
void BNO085_softReset(I2C_HandleTypeDef *hi2c);
uint8_t BNO085_resetReason(I2C_HandleTypeDef *hi2c);
void BNO085_sendCalibrateCommand(I2C_HandleTypeDef *hi2c, uint8_t thingToCalibrate);
float BNO085_qToFloat(int16_t fixedPointValue, uint8_t qPoint);

void BNO085_enableRotationVector(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports);
void BNO085_enableGameRotationVector(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports);
void BNO085_enableAccelerometer(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports);
void BNO085_enableLinearAccelerometer(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports);
void BNO085_enableGyro(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports);
void BNO085_enableMagnetometer(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports);
void BNO085_enableStepCounter(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports);

#endif // _BNO085_H
