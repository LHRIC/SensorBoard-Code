/**
 * @file BNO085.c
 * @brief IMU driver for BNO085
 * @date 2024-11-15
 * @author Samrut Gadde
 * @version 1.1
 */

#include "BNO085.h"
#ifdef DEBUG
#include <stdio.h>
#endif

/**
 * @brief Decode the sensor event and store it in the data buffer
 * @return sensor report_id from enum SH2_SensorReportID
 * @param uint8_t *data, of length 6
 * @param SH2_SensorEvent *event
 */
enum SH2_SensorReportID BNO085_DecodeSensorEvent(uint8_t *data, SH2_SensorEvent *event)
{
  uint8_t reportID = event->report_id;

  switch (reportID)
  {
  case SH2_ACCELEROMETER:
    data[0] = event->data.accelerometer.x & 0xFF;
    data[1] = (event->data.accelerometer.x >> 8) & 0xFF;
    data[2] = event->data.accelerometer.y & 0xFF;
    data[3] = (event->data.accelerometer.y >> 8) & 0xFF;
    data[4] = event->data.accelerometer.z & 0xFF;
    data[5] = (event->data.accelerometer.z >> 8) & 0xFF;
#ifdef DEBUG
    printf("Acceleration: x = %d, y = %d, z = %d\n", event->data.accelerometer.x, event->data.accelerometer.y, event->data.accelerometer.z);
#endif
    break;
  case SH2_GYROSCOPE_CALIBRATED:
    data[0] = event->data.gyroscope_calibrated.x & 0xFF;
    data[1] = (event->data.gyroscope_calibrated.x >> 8) & 0xFF;
    data[2] = event->data.gyroscope_calibrated.y & 0xFF;
    data[3] = (event->data.gyroscope_calibrated.y >> 8) & 0xFF;
    data[4] = event->data.gyroscope_calibrated.z & 0xFF;
    data[5] = (event->data.gyroscope_calibrated.z >> 8) & 0xFF;
#ifdef DEBUG
    printf("Gyroscope Calibrated: x = %d, y = %d, z = %d\n", event->data.gyroscope_calibrated.x, event->data.gyroscope_calibrated.y, event->data.gyroscope_calibrated.z);
#endif
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    data[0] = event->data.magnetometer_calibrated.x & 0xFF;
    data[1] = (event->data.magnetometer_calibrated.x >> 8) & 0xFF;
    data[2] = event->data.magnetometer_calibrated.y & 0xFF;
    data[3] = (event->data.magnetometer_calibrated.y >> 8) & 0xFF;
    data[4] = event->data.magnetometer_calibrated.z & 0xFF;
    data[5] = (event->data.magnetometer_calibrated.z >> 8) & 0xFF;
#ifdef DEBUG
    printf("Magnetometer Calibrated: x = %d, y = %d, z = %d\n", event->data.magnetometer_calibrated.x, event->data.magnetometer_calibrated.y, event->data.magnetometer_calibrated.z);
#endif
    break;
  case SH2_LINEAR_ACCELERATION:
    data[0] = event->data.linear_acceleration.x & 0xFF;
    data[1] = (event->data.linear_acceleration.x >> 8) & 0xFF;
    data[2] = event->data.linear_acceleration.y & 0xFF;
    data[3] = (event->data.linear_acceleration.y >> 8) & 0xFF;
    data[4] = event->data.linear_acceleration.z & 0xFF;
    data[5] = (event->data.linear_acceleration.z >> 8) & 0xFF;
#ifdef DEBUG
    printf("Linear Acceleration: x = %d, y = %d, z = %d\n", event->data.linear_acceleration.x, event->data.linear_acceleration.y, event->data.linear_acceleration.z);
#endif
    break;
  case SH2_ROTATION_VECTOR:
    data[0] = event->data.rotation_vector.i & 0xFF;
    data[1] = (event->data.rotation_vector.i >> 8) & 0xFF;
    data[2] = event->data.rotation_vector.j & 0xFF;
    data[3] = (event->data.rotation_vector.j >> 8) & 0xFF;
    data[4] = event->data.rotation_vector.k & 0xFF;
    data[5] = (event->data.rotation_vector.k >> 8) & 0xFF;
    data[6] = event->data.rotation_vector.real & 0xFF;
    data[7] = (event->data.rotation_vector.real >> 8) & 0xFF;
#ifdef DEBUG
    printf("Rotation Vector: i = %d, j = %d, k = %d, real = %d\n", event->data.rotation_vector.i, event->data.rotation_vector.j, event->data.rotation_vector.k, event->data.rotation_vector.real);
#endif
    break;
  default:
    break;
  }

  // // Cast to CommandResponse to see if it is a command response
  // SHTP_CommandResponse *response = (SHTP_CommandResponse *)event;

  // if (response->command == 0x04 || response->command == 0x84)
  // {
  //   // Initialize response
  //   return -1;
  // }

  return reportID;
}

// int BNO085_I2CPacketRead(I2C_HandleTypeDef *hi2c)
// {
//   // First read the header
//   SHTP_Header header = {0};

//   HAL_I2C_Master_Receive(hi2c, BNO085_ADDRESS, (uint8_t *)&header, sizeof(header), 1000);

//   // Unset continuation bit
//   header.length &= 0x8000;

//   uint8_t data[header.length - sizeof(header)];

//   HAL_I2C_Master_Receive(hi2c, BNO085_ADDRESS, (uint8_t *)&data, sizeof(data), 1000);
// }