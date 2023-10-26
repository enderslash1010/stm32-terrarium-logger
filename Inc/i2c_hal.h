
#ifndef I2C_HAL_H_
#define I2C_HAL_H_

#include <stdint.h>
#include <stdlib.h>
#include "delay.h"
#include "main.h"

typedef enum {
	I2C_OK,
	I2C_START_ERROR,
	I2C_ADDR_ERROR,
	I2C_DATA_ERROR,
	I2C_ERROR
} I2C_ERROR_CODE;

void i2c_init(I2C_TypeDef* I2C);

I2C_ERROR_CODE i2c_write(I2C_TypeDef* I2C, uint8_t addr, uint8_t* data, uint8_t len);
uint8_t* i2c_read(I2C_TypeDef* I2C, uint8_t addr, uint8_t numBytes);

#endif /* I2C_HAL_H_ */
