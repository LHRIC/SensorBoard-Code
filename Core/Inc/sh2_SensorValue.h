/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** 
 * @file sh2_SensorValue.h 
 * @author David Wheeler
 * @date 10 Nov 2015
 * @brief Support for converting sensor events (messages) into natural data structures.
 *
 */

#ifndef SH2_SENSORVALUE_H
#define SH2_SENSORVALUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "sh2.h"

/* Note on quaternion naming conventions:
 * Quaternions are values with four real components that are usually
 * interpreted as coefficients in the complex quantity, Q.
 *
 * As in, Q = W + Xi + Yj + Zk
 *
 * Where i, j and k represent the three imaginary dimensions.
 *
 * So W represents the Real components and X, Y and Z the Imaginary ones.
 *
 * In the Hillcrest datasheets and in this code, however, the four components
 * are named real, i, j and k, to make it explicit which is which.  If you 
 * need to translate these names into the "wxyz" or "xyzw" convention, then, the
 * appropriate mapping is this:
 *     w = real
 *     x = i
 *     y = j
 *     z = k
 */
	
/**
 * @brief Raw Accelerometer
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RawAccelerometer {
    /* Units are ADC counts */
    int16_t x;  /**< @brief [ADC counts] */
    int16_t y;  /**< @brief [ADC counts] */
    int16_t z;  /**< @brief [ADC counts] */

    /* Microseconds */
    uint32_t timestamp;  /**< @brief [uS] */
} sh2_RawAccelerometer_t;

// /**
//  * @brief Accelerometer
//  *
//  * See the SH-2 Reference Manual for more detail.
//  */
// typedef struct sh2_Accelerometer {
//     float x;
//     float y;
//     float z;
// } sh2_Accelerometer_t;

// /**
//  * @brief Raw gyroscope
//  *
//  * See the SH-2 Reference Manual for more detail.
//  */
// typedef struct sh2_RawGyroscope {
//     /* Units are ADC counts */
//     int16_t x;  /**< @brief [ADC Counts] */
//     int16_t y;  /**< @brief [ADC Counts] */
//     int16_t z;  /**< @brief [ADC Counts] */
//     int16_t temperature;  /**< @brief [ADC Counts] */

//     /* Microseconds */
//     uint32_t timestamp;  /**< @brief [uS] */
// } sh2_RawGyroscope_t;

// /**
//  * @brief Gyroscope
//  *
//  * See the SH-2 Reference Manual for more detail.
//  */
// typedef struct sh2_Gyroscope {
//     /* Units are rad/s */
//     float x;
//     float y;
//     float z;
// } sh2_Gyroscope_t;

// /**
//  * @brief Uncalibrated gyroscope
//  *
//  * See the SH-2 Reference Manual for more detail.
//  */
// typedef struct sh2_GyroscopeUncalibrated {
//     /* Units are rad/s */
//     float x;  /**< @brief [rad/s] */
//     float y;  /**< @brief [rad/s] */
//     float z;  /**< @brief [rad/s] */
//     float biasX;  /**< @brief [rad/s] */
//     float biasY;  /**< @brief [rad/s] */
//     float biasZ;  /**< @brief [rad/s] */
// } sh2_GyroscopeUncalibrated_t;

// /**
//  * @brief Raw Magnetometer
//  *
//  * See the SH-2 Reference Manual for more detail.
//  */
// typedef struct sh2_RawMagnetometer {
//     /* Units are ADC counts */
//     int16_t x;  /**< @brief [ADC Counts] */
//     int16_t y;  /**< @brief [ADC Counts] */
//     int16_t z;  /**< @brief [ADC Counts] */

//     /* Microseconds */
//     uint32_t timestamp;  /**< @brief [uS] */
// } sh2_RawMagnetometer_t;

// /**
//  * @brief Magnetic field
//  *
//  * See the SH-2 Reference Manual for more detail.
//  */
// typedef struct sh2_MagneticField {
//     /* Units are uTesla */
//     float x;  /**< @brief [uTesla] */
//     float y;  /**< @brief [uTesla] */
//     float z;  /**< @brief [uTesla] */
// } sh2_MagneticField_t;

// /**
//  * @brief Uncalibrated magnetic field
//  *
//  * See the SH-2 Reference Manual for more detail.
//  */
// typedef struct sh2_MagneticFieldUncalibrated {
//     /* Units are uTesla */
//     float x;  /**< @brief [uTesla] */
//     float y;  /**< @brief [uTesla] */
//     float z;  /**< @brief [uTesla] */
//     float biasX;  /**< @brief [uTesla] */
//     float biasY;  /**< @brief [uTesla] */
//     float biasZ;  /**< @brief [uTesla] */
// } sh2_MagneticFieldUncalibrated_t;

// /**
//  * @brief Rotation Vector with Accuracy
//  *
//  * See the SH-2 Reference Manual for more detail.
//  */
// typedef struct sh2_RotationVectorWAcc {
//     float i;  /**< @brief Quaternion component i */
//     float j;  /**< @brief Quaternion component j */
//     float k;  /**< @brief Quaternion component k */
//     float real;  /**< @brief Quaternion component, real */
//     float accuracy;  /**< @brief Accuracy estimate [radians] */
// } sh2_RotationVectorWAcc_t;

// /**
//  * @brief Rotation Vector
//  *
//  * See the SH-2 Reference Manual for more detail.
//  */
// typedef struct sh2_RotationVector {
//     float i;  /**< @brief Quaternion component i */
//     float j;  /**< @brief Quaternion component j */
//     float k;  /**< @brief Quaternion component k */
//     float real;  /**< @brief Quaternion component real */
// } sh2_RotationVector_t;

// /**
//  * @brief Gyro Integrated Rotation Vector
//  *
//  * See SH-2 Reference Manual for details.
//  */
// typedef struct sh2_GyroIntegratedRV {
//     float i;        /**< @brief Quaternion component i */
//     float j;        /**< @brief Quaternion component j */
//     float k;        /**< @brief Quaternion component k */
//     float real;     /**< @brief Quaternion component real */
//     float angVelX;  /**< @brief Angular velocity about x [rad/s] */
//     float angVelY;  /**< @brief Angular velocity about y [rad/s] */
//     float angVelZ;  /**< @brief Angular velocity about z [rad/s] */
// } sh2_GyroIntegratedRV_t;

typedef struct sh2_SensorValue {
    
    /** Which sensor produced this event. */
    uint8_t sensorId;

    /** @brief 8-bit unsigned integer used to track reports.
     *
     * The sequence number increments once for each report sent.  Gaps
     * in the sequence numbers indicate missing or dropped reports.
     */
    uint8_t sequence;

    /* Status of a sensor
     *   0 - Unreliable
     *   1 - Accuracy low
     *   2 - Accuracy medium
     *   3 - Accuracy high
     */
    uint8_t status; /**< @brief bits 7-5: reserved, 4-2: exponent delay, 1-0: Accuracy */

    uint64_t timestamp;  /**< [uS] */

    uint32_t delay; /**< @brief [uS] value is delay * 2^exponent (see status) */

    /** @brief Sensor Data
     *
     * Use the structure based on the value of the sensor
     * field.
     */
    union {
        sh2_RawAccelerometer_t rawAccelerometer;
        // sh2_Accelerometer_t accelerometer; 
        // sh2_Accelerometer_t linearAcceleration; 
        // sh2_Accelerometer_t gravity; 
        // sh2_RawGyroscope_t rawGyroscope; 
        // sh2_Gyroscope_t gyroscope; 
        // sh2_GyroscopeUncalibrated_t gyroscopeUncal; 
        // sh2_RawMagnetometer_t rawMagnetometer; 
        // sh2_MagneticField_t magneticField; 
        // sh2_MagneticFieldUncalibrated_t magneticFieldUncal; 
        // sh2_RotationVectorWAcc_t rotationVector; 
        // sh2_RotationVector_t gameRotationVector; 
        // sh2_RotationVectorWAcc_t geoMagRotationVector;
        // sh2_RotationVectorWAcc_t arvrStabilizedRV;
        // sh2_RotationVector_t arvrStabilizedGRV;
        // sh2_GyroIntegratedRV_t gyroIntegratedRV;
    } un;
} sh2_SensorValue_t;

int sh2_decodeSensorEvent(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);


#ifdef __cplusplus
} // extern "C"
#endif

#endif