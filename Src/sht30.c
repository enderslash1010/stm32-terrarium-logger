
#include "sht30.h"

SHT30_t sht30_init(I2C_TypeDef* I2C, uint32_t pclk1, uint8_t addr)
{
	SHT30_t sensor;
	sensor.I2C = I2C;
	sensor.addr = addr;

	i2c_init(I2C, pclk1); // Initialize I2C peripheral
	// The initialization time for I2C is greater than 1ms, so no need to wait for the SHT30 to enter idle mode (Maximum 1ms after power on)

	return sensor;
}

/*
 * Checksum Properties (SHT3x-DIS pg. 14)
 * Name: CRC-8
 * Width: 8 bit
 * Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 * Initialization: 0xFF
 * Reflect input: False
 * Reflect output: False
 * Final XOR: 0x0
 *
 * Example: CRC (0xBEEF) = 0x92
 */
static uint8_t compute_crc(const uint8_t* input, int len)
{
	const static uint8_t polynomial = 0x31;
	uint8_t crc = 0xFF;

	for (int i = 0; i < len; i++)
	{
		crc ^= input[i];
		for (int j = 0; j < 8; j++)
		{
			if (crc & 0x80) crc = ((crc << 1) ^ polynomial);
			else crc = (crc << 1);
		}
	}
	return crc;
}


// Converts the raw relative humidity value from the sensor to relative humidity
static double relative_humidity_conversion(uint16_t Srh)
{
	// RH = 100*((Srh)/(2^16 - 1))
	// Note: Srh must be used in decimal representation
	double result = 100 * ((double)Srh/0xFFFF);
	return result;
}

// Converts the raw temperature value from the sensor to temperature
static double temperature_conversion(uint16_t St, uint8_t isC)
{
	// T[Celsius] = -45 + 175*((St)/(2^16 - 1))
	// T[Fahrenheit] = -49 + 315*((St)/(2^16 - 1))
	// Note: St must be used in decimal representation
	double result;
	if (isC) result = -45 + (175 * ((double)St/0xFFFF));
	else result = -49 + (315 * ((double)St/0xFFFF));
	return result;
}

/*
 * Starts I2C Request for getting raw sensor values (6 bytes), uses I2C in interrupt mode
 * *Note*: In order to have I2C communications on both buses at the same time, wait for each bus to be free before accessing data buffer
 *      -> i.e. call sht30_get_raw_sensor(), then wait for I2C bus for that sensor to be free, then call sht30_convert_sensor()
 *
 * repeatability: 0->high, 1->medium, 2->low
 * clockStretching: 0->clock stretching disabled, 1->clock stretching enabled
 */
void sht30_get_raw_sensor(SHT30_t* sensor, uint8_t repeatability, uint8_t clockStretching, uint8_t data[6])
{
	//SensorValues_t result;

	uint16_t command; // command to be 'written' to sensor to start a measurement
	switch (repeatability)
	{
	case 0: // high
		if (clockStretching) command = 0x2C06;
		else command = 0x2400;
		break;
	case 1: // medium
		if (clockStretching) command = 0x2C0D;
		else command = 0x240B;
		break;
	case 2: // low
		if (clockStretching) command = 0x2C10;
		else command = 0x2416;
		break;
	}

	uint8_t commandPtr[2] = {command >> 8, command & 0xFF};
	i2c_write_dma(sensor->I2C, sensor->addr, commandPtr, 2);

	if (!clockStretching) delay_ms(15);
	i2c_read_dma(sensor->I2C, sensor->addr, data, 6);
}

// isC: 0->Celsius, 1->Fahrenheit
SensorValues_t sht30_convert_sensor(uint8_t data[6], uint8_t isC)
{
	SensorValues_t result;

	if (compute_crc(data, 3)) // Checksum failed for temperature
	{
		result.temperature = NAN; // NaN
	}
	else
	{
		uint16_t St = (data[0] << 8) | (data[1]);
		result.temperature = temperature_conversion(St, isC);
	}

	if (compute_crc(data + 3, 3)) // Checksum failed for humidity
	{
		result.relative_humidity = NAN; // NaN
	}
	else
	{
		uint16_t Srh = (data[3] << 8) | (data[4]);
		result.relative_humidity = relative_humidity_conversion(Srh);
	}

	return result;
}
