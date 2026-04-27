#include "main.h"

HAL_StatusTypeDef tcs3472_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef tcs3472_read_colors(I2C_HandleTypeDef *hi2c, uint16_t* colors);