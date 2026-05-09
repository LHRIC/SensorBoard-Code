#include "AS5600.h"
#include <stddef.h>

#define REG_ZMCO 0x00
#define REG_ZPOS_H 0x01
#define REG_ZPOS_L 0x02
#define REG_MPOS_H 0x03
#define REG_MPOS_L 0x04
#define REG_MANG_H 0x05
#define REG_MANG_L 0x06
#define REG_CONF_H 0x07
#define REG_CONF_L 0x08
#define REG_STATUS 0x0B
#define REG_RAW_ANGLE_H 0x0C
#define REG_RAW_ANGLE_L 0x0D
#define REG_ANGLE_H 0x0E
#define REG_ANGLE_L 0x0F
#define REG_AGC 0x1A
#define REG_MAGNITUDE_H 0x1B
#define REG_MAGNITUDE_L 0x1C

#define STATUS_MH (1 << 3)
#define STATUS_ML (1 << 4)
#define STATUS_MD (1 << 5)

as5600_t device;

static void read_reg(uint8_t addr, uint8_t *data, size_t len) {
  HAL_I2C_Mem_Read(device.i2c_handle, AS5600_I2C_ADDR, addr,
                   I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

// Redundant
void as5600_init(I2C_HandleTypeDef *i2c_handle) {
  device.i2c_handle = i2c_handle;
}

uint16_t as5600_read_angle(void) {
  uint8_t buffer[2];
  read_reg(REG_ANGLE_H, buffer, 2);

  uint16_t angle = ((buffer[0] & 0x0F) << 8) | buffer[1];

  return angle;
}

uint16_t as5600_read_angle_deg(void) {
  uint16_t angle_raw = as5600_read_angle();
  return (angle_raw * 360) / 4096;
}

float as5600_read_angle_deg_float(void) {
  uint16_t angle_int = as5600_read_angle();
  return angle_int * (360.0 / 4096.0);
}

bool as5600_is_valid(void) {
  uint8_t buffer;
  read_reg(REG_STATUS, &buffer, 1);

  return (buffer & STATUS_MD) != 0;
}
