#include "tcs3472.h"

// DEFINING REGISTERS
#define ENABLE_REG 0x00
#define ATIME_REG 0x01
#define WTIME_REG 0x03
#define AILTL_REG 0x04
#define AILTH_REG 0x05
#define AIHTL_REG 0x06
#define AIHTH_REG 0x07
#define PERS_REG 0x0C
#define CONFIG_REG 0x0D
#define CONTROL_REG 0x0F
#define ID_REG 0x12
#define STATUS_REG 0x13
#define COLOR_DATA_REGS_BEGIN 0x14
// (I am tired already...)
#define TCS3472_ADDR (0x29<<1)
#define TIMEOUT 100


HAL_StatusTypeDef tcs3472_init(I2C_HandleTypeDef *hi2c)
{
    uint8_t tx_data[2]; 

    // Checking if device is alive and ready
    if(HAL_I2C_IsDeviceReady(hi2c, TCS3472_ADDR, 3, TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    // CONFIGURE SENSOR

    // RGBC timing to 2.4ms
    tx_data[0] = (1<<7)|ATIME_REG;
    tx_data[1] = 0xFF;
    if(HAL_I2C_Master_Transmit(hi2c, TCS3472_ADDR, tx_data, 2, TIMEOUT)) {
        return HAL_ERROR;
    }

    // Config WLONG to 0
    tx_data[0] = (1<<7)|CONFIG_REG;
    tx_data[1] = 0x00;
    if(HAL_I2C_Master_Transmit(hi2c, TCS3472_ADDR, tx_data, 2, TIMEOUT)) {
        return HAL_ERROR;
    }

    // Gain to 1x
    tx_data[0] = (1<<7)|CONTROL_REG;
    tx_data[1] = 0x00;
    if(HAL_I2C_Master_Transmit(hi2c, TCS3472_ADDR, tx_data, 2, TIMEOUT)) {
        return HAL_ERROR;
    }

    // POWER ON THE SENSOR
    tx_data[0] = (1<<7)|ENABLE_REG;
    tx_data[1] = (1<<0)|(1<<1);
    if(HAL_I2C_Master_Transmit(hi2c, TCS3472_ADDR, tx_data, 2, TIMEOUT)) {
        return HAL_ERROR;
    }

    // Evertything is fine! return HAL_OK
    return HAL_OK;
}

HAL_StatusTypeDef tcs3472_read_colors(I2C_HandleTypeDef *hi2c, uint16_t *colors) // colors must be array with 4 items
{
    uint8_t tx_data[1];
    uint8_t rx_data[8];

    // wait for data to be ready
    while( !((rx_data[0]>>0)&1) )
    {
        tx_data[0] = (1<<7)|STATUS_REG;
        if(HAL_I2C_Master_Transmit(hi2c, TCS3472_ADDR, tx_data, 1, TIMEOUT)) {
            return HAL_ERROR;
        }
        if(HAL_I2C_Master_Receive(hi2c, TCS3472_ADDR, rx_data, 1, TIMEOUT)){
            return HAL_ERROR;
        }
    }

    // read colors data
    tx_data[0] = (1<<7)|(1<<5)|COLOR_DATA_REGS_BEGIN;
    if(HAL_I2C_Master_Transmit(hi2c, TCS3472_ADDR, tx_data, 1, TIMEOUT)) {
        return HAL_ERROR;
    }
    if(HAL_I2C_Master_Receive(hi2c, TCS3472_ADDR, rx_data, 8, TIMEOUT)) {
        return HAL_ERROR;
    }

    // convert colors data
    colors[0] = (uint16_t)rx_data[0]|((uint16_t)rx_data[1]<<8);
    colors[1] = (uint16_t)rx_data[2]|((uint16_t)rx_data[3]<<8);
    colors[2] = (uint16_t)rx_data[4]|((uint16_t)rx_data[5]<<8);
    colors[3] = (uint16_t)rx_data[6]|((uint16_t)rx_data[7]<<8);


    // If
    return HAL_OK;
}