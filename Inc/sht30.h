/*
 * sht30.h
 *
 *  Created on: Oct 24, 2023
 *      Author: ender
 */

#ifndef SHT30_H_
#define SHT30_H_

#include "main.h"
#include "i2c_hal.h"
#include "math.h" // Included for NAN definition

typedef struct SHT30
{
	I2C_TypeDef* I2C;
	uint8_t addr;
} SHT30_t;

typedef struct SensorValues
{
	double relative_humidity;
	double temperature;
} SensorValues_t;

SHT30_t sht30_init(I2C_TypeDef* I2C, uint32_t pclk1, uint8_t addr);

void sht30_get_raw_sensor(SHT30_t* sensor, uint8_t repeatability, uint8_t clockStretching, uint8_t data[6]);
SensorValues_t sht30_convert_sensor(uint8_t data[6], uint8_t isC);

#endif /* SHT30_H_ */
