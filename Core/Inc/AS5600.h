#ifndef AS5600_H
#define AS5600_H

#include "stm32f3xx.h"
#include "stm32f3xx_hal_i2c.h"

#include <stdbool.h>
#include <stdint.h>

#define AS5600_I2C_ADDR (0x36 << 1)

typedef struct {
  I2C_HandleTypeDef *i2c_handle;
} as5600_t;

void as5600_init(I2C_HandleTypeDef *i2c_handle);

bool as5600_is_valid(void);

uint16_t as5600_read_angle(void);
uint16_t as5600_read_angle_deg(void);

float as5600_read_angle_deg_float(void);

#endif
