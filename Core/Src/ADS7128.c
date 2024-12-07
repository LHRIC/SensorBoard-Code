#include "ADS7128.h"
#include <cstdint>

uint8_t ADS7128_Initialize(ADS7128 *dev, I2C_HandleTypeDef *i2cHandle, uint8_t num_samples)
{

    /* Set struct parameters */
    dev->i2cHandle = i2cHandle;

    dev->acc_mps2[0] = 0.0f;
    dev->acc_mps2[1] = 0.0f;
    dev->acc_mps2[2] = 0.0f;

    /* Store number of transaction errors (to be returned at end of function) */
    uint8_t errNum = 0;
    HAL_StatusTypeDef status;

    /*
     * Check device, mems, and part IDs (DATASHEET PAGE 30)
     */
    uint8_t regData;

    status = ADS7128_ReadRegister(dev, ADS7128_REG_SYSTEM_STATUS, &regData);
    errNum += (status != HAL_OK);

    if (regData != ADS7128_SYSTEM_STATUS)
    {

        return 255;
    }

    status = ADS7128_ReadRegister(dev, ADS7128_REG_GENERAL_CFG, &regData);
    errNum += (status != HAL_OK);

    if (regData != ADS7128_GENERAL_CFG)
    {

        return 255;
    }

    status = ADS7128_ReadRegister(dev, ADS7128_REG_DATA_CFG, &regData);
    errNum += (status != HAL_OK);

    if (regData != ADS7128_DATA_CFG)
    {

        return 255;
    }

    /*
     * Set number of filters for average sampling (OSR) (DATASHEET PAGE 35)
     */
    regData = num_samples & 0x07;

    status = ADS7128_WriteRegister(dev, ADS7128_REG_OSR_CFG, &regData);
    errNum += (status != HAL_OK);

    /*
     * Start conversation (DATASHEET PAGE 16)
     */
    regData = CNVST_BIT;

    status = ADS7128_WriteRegister(dev, ADS7128_REG_GENERAL_CFG, &regData);
    errNum += (status != HAL_OK);

    /* Return number of errors (0 if successful initialisation) */
    return errNum;
}

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ADS7128_ReadRegister(ADS7128 *dev, uint8_t reg, uint8_t *data)
{

    return HAL_I2C_Mem_Read(dev->i2cHandle, ADS7128_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADS7128_ReadRecentRegister(ADS7128 *dev, uint8_t channel_num, uint16_t *data)
{
    uint8_t reg_lsb;
    uint8_t reg_msb;
    switch (channel_num)
    {
    case 0:
        reg_lsb = ADS7128_REG_RECENT_CH0_LSB;
        reg_msb = ADS7128_REG_RECENT_CH0_MSB;
        break;
    case 1:
        reg_lsb = ADS7128_REG_RECENT_CH1_LSB;
        reg_msb = ADS7128_REG_RECENT_CH1_MSB;
        break;
    case 2:
        reg_lsb = ADS7128_REG_RECENT_CH2_LSB;
        reg_msb = ADS7128_REG_RECENT_CH2_MSB;
        break;
    case 3:
        reg_lsb = ADS7128_REG_RECENT_CH3_LSB;
        reg_msb = ADS7128_REG_RECENT_CH3_MSB;
        break;
    case 4:
        reg_lsb = ADS7128_REG_RECENT_CH4_LSB;
        reg_msb = ADS7128_REG_RECENT_CH4_MSB;
        break;
    case 5:
        reg_lsb = ADS7128_REG_RECENT_CH5_LSB;
        reg_msb = ADS7128_REG_RECENT_CH5_MSB;
        break;
    case 6:
        reg_lsb = ADS7128_REG_RECENT_CH6_LSB;
        reg_msb = ADS7128_REG_RECENT_CH6_MSB;
        break;
    case 7:
        reg_lsb = ADS7128_REG_RECENT_CH7_LSB;
        reg_msb = ADS7128_REG_RECENT_CH7_MSB;
        break;
    default:
        printf("bruh");
        break;
    }

    // read lsb and msb into temp_data
    if (HAL_I2C_Mem_Read(dev->i2cHandle, ADS7128_I2C_ADDR, reg_lsb, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("Error reading Register %d LSB", channel_num);
    }
    if (HAL_I2C_Mem_Read(dev->i2cHandle, ADS7128_I2C_ADDR, reg_msb, I2C_MEMADD_SIZE_8BIT, data[1], 1, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("Error reading Register %d MSB", channel_num);
    }
    return HAL_OK;
}

HAL_StatusTypeDef ADS7128_ReadRegisters(ADS7128 *dev, uint8_t reg, uint8_t *data, uint8_t length)
{

    return HAL_I2C_Mem_Read(dev->i2cHandle, ADS7128_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADS7128_WriteRegister(ADS7128 *dev, uint8_t reg, uint8_t *data)
{

    return HAL_I2C_Mem_Write(dev->i2cHandle, ADS7128_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}