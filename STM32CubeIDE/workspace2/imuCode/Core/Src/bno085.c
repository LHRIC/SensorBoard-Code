#include "stm32f4xx_hal.h"  // STM32F4 series
#include "stm32f4xx_hal_gpio.h"
#include "bno085.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_hal_i2c.h"
#include <math.h>

// Global Variables
uint8_t shtpHeader[4]; // Each packet has a header of 4 bytes
uint8_t shtpData[MAX_PACKET_SIZE];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; // Sequence numbers for 6 communication channels
uint8_t commandSequenceNumber = 0; // Commands have a seqNum as well
uint32_t metaData[MAX_METADATA_SIZE]; // Metadata storage

// Raw sensor values from input reports
uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
uint16_t stepCount;
uint32_t timeStamp;
uint8_t stabilityClassifier;
uint8_t activityClassifier;
uint8_t *_activityConfidences; // Array for activity confidence levels
uint8_t calibrationStatus; // Calibration status

int16_t rotationVector_Q1 = 14;
int16_t accelerometer_Q1 = 8;
int16_t linear_accelerometer_Q1 = 8;
int16_t gyro_Q1 = 9;
int16_t magnetometer_Q1 = 4;


void BNO085_GPIO_I2C_Initialization(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable clocks for GPIO
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure I2C pins: PB6 -> I2C1_SCL, PB7 -> I2C1_SDA
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PA8 -> BNO085_PS0/WAKE (output)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    // PC9 -> BNO085_RST (output)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // Assert reset
    HAL_Delay(10); // Delay for 10ms
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Deassert reset
    HAL_Delay(100); // Additional delay for sensor initialization

    // PC8 -> BNO085_INT (input)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BNO085_INT_PORT, BNO085_INT_PIN, 1);
}


int BNO085_Initialization(I2C_HandleTypeDef *hi2c) {
    BNO085_GPIO_I2C_Initialization();

    printf("Checking BNO085...");

    // Configure the BNO085 for I2C communication
    WAKE_HIGH(); // Before boot up the PS0/WAK pin must be high to enter I2C mode
    RESET_LOW(); // Reset BNO085
    HAL_Delay(300); // Minimum length not specified in datasheet
    RESET_HIGH(); // Bring out of reset

    // Wait for the BNO085 to be ready
    HAL_Delay(50); // Wait for the BNO085 to be ready

    // Check communication with the device
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // Request the product ID
    shtpData[1] = 0; // Reserved

//    // Transmit packet on control channel, 2 bytes
    if (!BNO085_sendPacket(hi2c, CHANNEL_CONTROL, 2))
        return 1; // Something went wrong

    // Wait for the response
    if (BNO085_receivePacket(hi2c)) {
        if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
            printf("BNO085 who_am_i = 0x%02x...ok\n\n", shtpData[0]);
            return 0; // Sensor OK
        }
    }

    printf("BNO085 Not OK: 0x%02x Should be 0x%02x\n", shtpData[0], SHTP_REPORT_PRODUCT_ID_RESPONSE);
    return 1; // Something went wrong
}

void BNO085_parseCommandReport(void) {
    if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE) {
        // The BNO085 responds with this report to command requests. It's up to us to remember which command we issued.
        uint8_t command = shtpData[2]; // This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE) {
            calibrationStatus = shtpData[5]; // R0 - Status (0 = success, non-zero = fail)
        }
    } else {
        // This sensor report ID is unhandled.
        // See the reference manual to add additional feature reports as needed
    }

    // TODO: Additional feature reports may be strung together. Parse them all.
}

void BNO085_parseInputReport(void) {
    // Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
    dataLength &= ~(1 << 15); // Clear the MSbit. This bit indicates if this package is a continuation of the last.
    // Ignore it for now. TODO: Catch this as an error and exit

    dataLength -= 4; // Remove the header bytes from the data count

    timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | (shtpData[3] << (8 * 2)) | (shtpData[2] << (8 * 1)) | (shtpData[1] << (8 * 0));

    uint8_t status = shtpData[7] & 0x03; // Get status bits
    uint16_t data1 = (uint16_t)shtpData[10] << 8 | shtpData[9];
    uint16_t data2 = (uint16_t)shtpData[12] << 8 | shtpData[11];
    uint16_t data3 = (uint16_t)shtpData[14] << 8 | shtpData[13];
    uint16_t data4 = 0;
    uint16_t data5 = 0;

    if (dataLength > 14) {
        data4 = (uint16_t)shtpData[16] << 8 | shtpData[15];
    }
    if (dataLength > 16) {
        data5 = (uint16_t)shtpData[18] << 8 | shtpData[17];
    }

    // Store these generic values to their proper global variable
    switch (shtpData[5]) {
        case SENSOR_REPORTID_ACCELEROMETER: {
            accelAccuracy = status;
            rawAccelX = data1;
            rawAccelY = data2;
            rawAccelZ = data3;
            break;
        }
        case SENSOR_REPORTID_LINEAR_ACCELERATION: {
            accelLinAccuracy = status;
            rawLinAccelX = data1;
            rawLinAccelY = data2;
            rawLinAccelZ = data3;
            break;
        }
        case SENSOR_REPORTID_GYROSCOPE: {
            gyroAccuracy = status;
            rawGyroX = data1;
            rawGyroY = data2;
            rawGyroZ = data3;
            break;
        }
        case SENSOR_REPORTID_MAGNETIC_FIELD: {
            magAccuracy = status;
            rawMagX = data1;
            rawMagY = data2;
            rawMagZ = data3;
            break;
        }
        case SENSOR_REPORTID_ROTATION_VECTOR:
        case SENSOR_REPORTID_GAME_ROTATION_VECTOR: {
            quatAccuracy = status;
            rawQuatI = data1;
            rawQuatJ = data2;
            rawQuatK = data3;
            rawQuatReal = data4;
            rawQuatRadianAccuracy = data5; // Only available on rotation vector, not game rotation vector
            break;
        }
//        case SENSOR_REPORTID_STEP_COUNTER: {
//            stepCount = data3; // Bytes 8/9
//            break;
//        }
        case SENSOR_REPORTID_STABILITY_CLASSIFIER: {
            stabilityClassifier = shtpData[5 + 4]; // Byte 4 only
            break;
        }
        case SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER: {
            activityClassifier = shtpData[5 + 5]; // Most likely state

            // Load activity classification confidences into the array
            for (uint8_t x = 0; x < 9; x++) {
                // Hardcoded to max of 9. TODO: Bring in array size
                _activityConfidences[x] = shtpData[11 + x]; // 5 bytes of timestamp, byte 6 is first confidence byte
            }
            break;
        }
        case SHTP_REPORT_COMMAND_RESPONSE: {
            // The BNO085 responds with this report to command requests. It's up to us to remember which command we issued.
            uint8_t command = shtpData[5 + 2]; // This is the Command byte of the response

            if (command == COMMAND_ME_CALIBRATE) {
                calibrationStatus = shtpData[5 + 5]; // R0 - Status (0 = success, non-zero = fail)
            }
            break;
        }
        default: {
            // This sensor report ID is unhandled.
            // See the reference manual to add additional feature reports as needed
        }
    }

    // TODO: Additional feature reports may be strung together. Parse them all.
}


void BNO085_setFeatureCommand(I2C_HandleTypeDef *hi2c, uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig) {
    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;  // Set feature command. Reference page 55
    shtpData[1] = reportID;                         // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    shtpData[2] = 0;                                // Feature flags
    shtpData[3] = 0;                                // Change sensitivity (LSB)
    shtpData[4] = 0;                                // Change sensitivity (MSB)
    shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  // Report interval (LSB) in microseconds. 0x7A120 = 500ms
    shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  // Report interval
    shtpData[7] = (microsBetweenReports >> 16) & 0xFF; // Report interval
    shtpData[8] = (microsBetweenReports >> 24) & 0xFF; // Report interval (MSB)
    shtpData[9] = 0;                                // Batch Interval (LSB)
    shtpData[10] = 0;                               // Batch Interval
    shtpData[11] = 0;                               // Batch Interval
    shtpData[12] = 0;                               // Batch Interval (MSB)
    shtpData[13] = (specificConfig >> 0) & 0xFF;    // Sensor-specific config (LSB)
    shtpData[14] = (specificConfig >> 8) & 0xFF;    // Sensor-specific config
    shtpData[15] = (specificConfig >> 16) & 0xFF;   // Sensor-specific config
    shtpData[16] = (specificConfig >> 24) & 0xFF;   // Sensor-specific config (MSB)

    // Transmit packet on control channel, 17 bytes
    BNO085_sendPacket(hi2c, CHANNEL_CONTROL, 17);
}



void BNO085_sendCommand(I2C_HandleTypeDef *hi2c, uint8_t command) {
    shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; // Command Request
    shtpData[1] = commandSequenceNumber++;     // Increments automatically each function call
    shtpData[2] = command;                     // Command

    // Caller must set these if needed
    /*shtpData[3] = 0; //P0
    shtpData[4] = 0; //P1
    shtpData[5] = 0; //P2
    shtpData[6] = 0;
    shtpData[7] = 0;
    shtpData[8] = 0;
    shtpData[9] = 0;
    shtpData[10] = 0;
    shtpData[11] = 0;*/

    // Transmit packet on control channel, 12 bytes
    BNO085_sendPacket(hi2c, CHANNEL_CONTROL, 12);
}


int BNO085_dataAvailable(I2C_HandleTypeDef *hi2c) {
    // If we have an interrupt pin connection available, check if data is available
    if (LL_GPIO_IsInputPinSet(BNO085_INT_PORT, BNO085_INT_PIN) == 1)
        return 0; // Data is not available

    if (BNO085_receivePacket(hi2c)) {
        // Check to see if this packet is a sensor reporting its data to us
        if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP) {
            BNO085_parseInputReport(); // This will update the rawAccelX, etc. variables depending on which feature report is found
            return 1;
        } else if (shtpHeader[2] == CHANNEL_CONTROL) {
            BNO085_parseCommandReport(); // This will update responses to commands, calibrationStatus, etc.
            return 1;
        }
    }
    return 0;
}


void BNO085_enableGameRotationVector(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports) {
    BNO085_setFeatureCommand(hi2c, SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports, 0);
}

void BNO085_enableRotationVector(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports) {
    BNO085_setFeatureCommand(hi2c, SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports, 0);
}

void BNO085_enableAccelerometer(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports) {
    BNO085_setFeatureCommand(hi2c, SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports, 0);
}

void BNO085_enableLinearAccelerometer(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports) {
    BNO085_setFeatureCommand(hi2c, SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports, 0);
}

void BNO085_enableGyro(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports) {
    BNO085_setFeatureCommand(hi2c, SENSOR_REPORTID_GYROSCOPE, timeBetweenReports, 0);
}
void BNO085_softReset(I2C_HandleTypeDef *hi2c) {
    shtpData[0] = 1; // Reset

    // Attempt to start communication with sensor
    BNO085_sendPacket(hi2c, CHANNEL_EXECUTABLE, 1); // Transmit packet on channel 1, 1 byte

    // Read all incoming data and flush it
    HAL_Delay(50);
    while (BNO085_receivePacket(hi2c) == 1);
    HAL_Delay(50);
    while (BNO085_receivePacket(hi2c) == 1);
}

uint8_t BNO085_resetReason(I2C_HandleTypeDef *hi2c) {
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // Request the product ID and reset info
    shtpData[1] = 0;                              // Reserved

    // Transmit packet on control channel, 2 bytes
    BNO085_sendPacket(hi2c, CHANNEL_CONTROL, 2);

    // Now we wait for response
    if (BNO085_receivePacket(hi2c) == 1) {
        if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
            return shtpData[1];
        }
    }

    return 0;
}

int16_t BNO085_getQ3(I2C_HandleTypeDef *hi2c, uint16_t recordID) {
    // Q3 is always the upper 16 bits of word 8
    return BNO085_readFRSword(hi2c, recordID, 8) >> 16; // Get word 8, upper 16 bits
}

float BNO085_getResolution(I2C_HandleTypeDef *hi2c, uint16_t recordID) {
    // The resolution Q value are 'the same as those used in the sensor's input report'
    // This should be Q1.
    int16_t Q = BNO085_getQ1(recordID); // Adjusted to match the function definition

    // Resolution is always word 2
    uint32_t value = BNO085_readFRSword(hi2c, recordID, 2); // Get word 2

    return BNO085_qToFloat(value, Q);
}


float BNO085_getRange(I2C_HandleTypeDef *hi2c, uint16_t recordID) {
    // The resolution Q value are 'the same as those used in the sensor's input report'
    // This should be Q1.
    int16_t Q = BNO085_getQ1(recordID);

    // Range is always word 1
    uint32_t value = BNO085_readFRSword(hi2c, recordID, 1); // Get word 1

    return BNO085_qToFloat(value, Q);
}

uint32_t BNO085_readFRSword(I2C_HandleTypeDef *hi2c, uint16_t recordID, uint8_t wordNumber) {
    if (BNO085_readFRSdata(hi2c, recordID, wordNumber, 1) == 1) // Get word number, just one word in length from FRS
        return metaData[0];                                    // Return this one word

    return 0; // Error
}

void BNO085_frsReadRequest(I2C_HandleTypeDef *hi2c, uint16_t recordID, uint16_t readOffset, uint16_t blockSize) {
    shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; // FRS Read Request
    shtpData[1] = 0;                            // Reserved
    shtpData[2] = (readOffset >> 0) & 0xFF;     // Read Offset LSB
    shtpData[3] = (readOffset >> 8) & 0xFF;     // Read Offset MSB
    shtpData[4] = (recordID >> 0) & 0xFF;       // FRS Type LSB
    shtpData[5] = (recordID >> 8) & 0xFF;       // FRS Type MSB
    shtpData[6] = (blockSize >> 0) & 0xFF;      // Block size LSB
    shtpData[7] = (blockSize >> 8) & 0xFF;      // Block size MSB

    // Transmit packet on control channel, 8 bytes
    BNO085_sendPacket(hi2c, CHANNEL_CONTROL, 8);
}

int BNO085_readFRSdata(I2C_HandleTypeDef *hi2c, uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead) {
    uint8_t spot = 0;

    // First we send a Flash Record System (FRS) request
    BNO085_frsReadRequest(hi2c, recordID, startLocation, wordsToRead); // From startLocation of record, read a # of words

    // Read bytes until FRS reports that the read is complete
    while (1) {
        // Now we wait for response
        while (1) {
            uint8_t counter = 0;
            while (BNO085_receivePacket(hi2c) == 0) {
                if (counter++ > 100)
                    return 0; // Give up
                HAL_Delay(1);
            }

            // We have the packet, inspect it for the right contents
            // Report ID should be 0xF3 and the FRS types should match the thing we requested
            if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE) {
                if (((uint16_t)shtpData[13] << 8 | shtpData[12]) == recordID) {
                    break; // This packet is one we are looking for
                }
            }
        }

        uint8_t dataLength = shtpData[1] >> 4;
        uint8_t frsStatus = shtpData[1] & 0x0F;

        uint32_t data0 = (uint32_t)shtpData[7] << 24 | (uint32_t)shtpData[6] << 16 | (uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
        uint32_t data1 = (uint32_t)shtpData[11] << 24 | (uint32_t)shtpData[10] << 16 | (uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];

        // Record these words to the metaData array
        if (dataLength > 0) {
            metaData[spot++] = data0;
        }
        if (dataLength > 1) {
            metaData[spot++] = data1;
        }

        if (spot >= MAX_METADATA_SIZE) {
            printf("metaData array overrun. Returning.");
            return 1; // We have run out of space in our array. Bail.
        }

        if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7) {
            return 1; // FRS status is read completed! We're done!
        }
    }
}

float BNO085_getQuatI() {
    return BNO085_qToFloat(rawQuatI, rotationVector_Q1);
}

float BNO085_getQuatJ() {
    return BNO085_qToFloat(rawQuatJ, rotationVector_Q1);
}

float BNO085_getQuatK() {
    return BNO085_qToFloat(rawQuatK, rotationVector_Q1);
}

float BNO085_getQuatReal() {
    return BNO085_qToFloat(rawQuatReal, rotationVector_Q1);
}

float BNO085_getQuatRadianAccuracy() {
    return BNO085_qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
}

uint8_t BNO085_getQuatAccuracy() {
    return quatAccuracy;
}

float BNO085_getAccelX() {
    return BNO085_qToFloat(rawAccelX, accelerometer_Q1);
}

float BNO085_getAccelY() {
    return BNO085_qToFloat(rawAccelY, accelerometer_Q1);
}

float BNO085_getAccelZ() {
    return BNO085_qToFloat(rawAccelZ, accelerometer_Q1);
}

uint8_t BNO085_getAccelAccuracy() {
    return accelAccuracy;
}

float BNO085_getLinAccelX() {
    return BNO085_qToFloat(rawLinAccelX, linear_accelerometer_Q1);
}

// Return the linear acceleration component
float BNO085_getLinAccelY() {
    return BNO085_qToFloat(rawLinAccelY, linear_accelerometer_Q1);
}

// Return the linear acceleration component
float BNO085_getLinAccelZ() {
    return BNO085_qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
}

uint8_t BNO085_getLinAccelAccuracy() {
    return accelLinAccuracy;
}

float BNO085_getMagX() {
    return BNO085_qToFloat(rawMagX, magnetometer_Q1);
}

float BNO085_getMagY() {
    return BNO085_qToFloat(rawMagY, magnetometer_Q1);
}

float BNO085_getMagZ() {
    return BNO085_qToFloat(rawMagZ, magnetometer_Q1);
}

uint8_t BNO085_getMagAccuracy() {
    return magAccuracy;
}

//uint16_t BNO085_getStepCount() {
//    return stepCount;
//}

uint8_t BNO085_getStabilityClassifier() {
    return stabilityClassifier;
}

uint8_t BNO085_getActivityClassifier() {
    return activityClassifier;
}

// Return the time stamp
uint32_t BNO085_getTimeStamp() {
    return timeStamp;
}



float BNO085_qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	return fixedPointValue * powf(2, qPoint * -1);
}

void BNO085_calibratePlanarAccelerometer(I2C_HandleTypeDef *hi2c) {
    BNO085_sendCalibrateCommand(hi2c, CALIBRATE_PLANAR_ACCEL);
}

void BNO085_calibrateAll(I2C_HandleTypeDef *hi2c) {
    BNO085_sendCalibrateCommand(hi2c, CALIBRATE_ACCEL_GYRO_MAG);
}

void BNO085_endCalibration(I2C_HandleTypeDef *hi2c) {
    BNO085_sendCalibrateCommand(hi2c, CALIBRATE_STOP); // Disables all calibrations
}

int BNO085_calibrationComplete(void) {
    if (calibrationStatus == 0)
        return 1;
    return 0;
}

void BNO085_enableMagnetometer(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports) {
    BNO085_setFeatureCommand(hi2c, SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports, 0);
}

void BNO085_enableStepCounter(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports) {
    BNO085_setFeatureCommand(hi2c, SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports, 0);
}

void BNO085_enableStabilityClassifier(I2C_HandleTypeDef *hi2c, uint16_t timeBetweenReports) {
    BNO085_setFeatureCommand(hi2c, SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports, 0);
}

void BNO085_calibrateAccelerometer(I2C_HandleTypeDef *hi2c) {
    BNO085_sendCalibrateCommand(hi2c, CALIBRATE_ACCEL);
}
void BNO085_calibrateGyro(I2C_HandleTypeDef *hi2c) {
    BNO085_sendCalibrateCommand(hi2c, CALIBRATE_GYRO);
}

void BNO085_calibrateMagnetometer(I2C_HandleTypeDef *hi2c) {
    BNO085_sendCalibrateCommand(hi2c, CALIBRATE_MAG);
}

void BNO085_sendCalibrateCommand(I2C_HandleTypeDef *hi2c, uint8_t thingToCalibrate) {
    for (uint8_t x = 3; x < 12; x++) // Clear this section of the shtpData array
        shtpData[x] = 0;

    if (thingToCalibrate == CALIBRATE_ACCEL)
        shtpData[3] = 1;
    else if (thingToCalibrate == CALIBRATE_GYRO)
        shtpData[4] = 1;
    else if (thingToCalibrate == CALIBRATE_MAG)
        shtpData[5] = 1;
    else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
        shtpData[7] = 1;
    else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG) {
        shtpData[3] = 1;
        shtpData[4] = 1;
        shtpData[5] = 1;
    }
    // No action needed for CALIBRATE_STOP, bytes are already set to zero

    // Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
    calibrationStatus = 1;

    // Using this shtpData packet, send a command
    BNO085_sendCommand(hi2c, COMMAND_ME_CALIBRATE);
}

void BNO085_requestCalibrationStatus(I2C_HandleTypeDef *hi2c) {
    for (uint8_t x = 3; x < 12; x++) // Clear this section of the shtpData array
        shtpData[x] = 0;

    shtpData[6] = 0x01; // P3 - 0x01 - Subcommand: Get ME Calibration

    // Using this shtpData packet, send a command
    BNO085_sendCommand(hi2c, COMMAND_ME_CALIBRATE);
}

void BNO085_saveCalibration(I2C_HandleTypeDef *hi2c) {
    for (uint8_t x = 3; x < 12; x++) // Clear this section of the shtpData array
        shtpData[x] = 0;

    // Using this shtpData packet, send a command
    BNO085_sendCommand(hi2c, COMMAND_DCD); // Save DCD command
}

int BNO085_waitForI2C(void) {
    for (uint32_t counter = 0; counter < 0xffffffff; counter++) {
        if (LL_GPIO_IsInputPinSet(BNO085_INT_PORT, BNO085_INT_PIN) == 0) {
            return 1; // Data available
        }
    }
    return 0; // Data not available
}


// Function to receive packet
int BNO085_receivePacket(I2C_HandleTypeDef *hi2c) {
    if (LL_GPIO_IsInputPinSet(BNO085_INT_PORT, BNO085_INT_PIN) == 1)
        return 0; // Data is not available

    // Get the first four bytes (packet header)
    HAL_I2C_Master_Receive(hi2c, BNO085_ADDRESS, shtpHeader, 4, HAL_MAX_DELAY);

    uint8_t packetLSB = shtpHeader[0];
    uint8_t packetMSB = shtpHeader[1];
  //  uint8_t channelNumber = shtpHeader[2];
   // uint8_t sequenceNumber = shtpHeader[3]; // Not sure if we need to store this or not

    // Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t)packetMSB << 8 | packetLSB);
    dataLength &= 0x7fff; // Clear the MSbit. Ignore continuation for now.

    if (dataLength == 0) {
        // Packet is empty
        return 0; // All done
    }
    dataLength -= 4; // Remove the header bytes from the data count

    // Read incoming data into the shtpData array
    if (dataLength > MAX_PACKET_SIZE) dataLength = MAX_PACKET_SIZE; // Prevent overflow

    HAL_I2C_Master_Receive(hi2c, BNO085_ADDRESS, shtpData, dataLength, HAL_MAX_DELAY);

    return 1; // We're done!
}


int BNO085_sendPacket(I2C_HandleTypeDef *hi2c, uint8_t channelNumber, uint8_t dataLength) {
    uint8_t packetLength = dataLength + 4; // Add four bytes for the header

    // Wait for BNO085 to indicate it is available for communication
    if (LL_GPIO_IsInputPinSet(BNO085_INT_PORT, BNO085_INT_PIN) == 1)
        return 0; // Data is not available

    uint8_t packet[packetLength];

    // Create the 4-byte packet header
    packet[0] = packetLength & 0xFF;         // Packet length LSB
    packet[1] = packetLength >> 8;           // Packet length MSB
    packet[2] = channelNumber;               // Channel number
    packet[3] = sequenceNumber[channelNumber]++;  // Send the sequence number, increments with each packet sent

    // Copy the user's data into the packet
    for (uint8_t i = 0; i < dataLength; i++) {
        packet[4 + i] = shtpData[i];
    }

    // Send the packet over I2C
    HAL_I2C_Master_Transmit(hi2c, BNO085_ADDRESS, packet, packetLength, HAL_MAX_DELAY);

    return 1;
}


