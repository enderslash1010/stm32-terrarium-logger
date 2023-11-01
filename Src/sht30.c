
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
 * Checksum Properties
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

/*
static uint8_t compute_crc()
{
	// TODO
	return 0;
}
*/

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

static void start_measurement(SHT30_t* sensor, uint16_t command)
{
	// I2C write, with 16-bit measurement command
	uint8_t commandPtr[] = {command >> 8, command & 0xFF};
	i2c_write_it(sensor->I2C, sensor->addr, commandPtr, 2);
}

static uint8_t* read_measurement(SHT30_t* sensor)
{
	// I2C read, with 16-bit temperature value, checksum, 16-bit humidity value, checksum
	uint8_t* data = i2c_read_it(sensor->I2C, sensor->addr, 6);
	return data;
}

/*
 * repeatability: 0->high, 1->medium, 2->low
 * clockStretching: 0->clock stretching disabled, 1->clock stretching enabled
 * isC: 0->Celsius, 1->Fahrenheit
 */
SensorValues_t sht30_get_sensor_value(SHT30_t* sensor, uint8_t repeatability, uint8_t clockStretching, uint8_t isC)
{
	SensorValues_t result;

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

	start_measurement(sensor, command); // Send command to sensor to start measurement

	uint8_t* data;
	if (clockStretching) data = read_measurement(sensor); // If clock stretching is enabled, no need to wait for measurement to complete, as the sensor will pull SCL low until it's completed
	else {
		// Clock stretching disabled, need to wait a bit until read request is sent to avoid getting a NACK from the sensor
		delay_ms(15); // 15ms delay for all repeatability types for simplicity
		data = read_measurement(sensor);
	}

	// Compose raw sensor values into 16-bit St and Srh variables
	uint16_t St = (data[0] << 8) | (data[1]);
	uint16_t Srh = (data[3] << 8) | (data[4]);

	// Convert raw sensor values to relative humidity and temperature
	result.relative_humidity = relative_humidity_conversion(Srh);
	result.temperature = temperature_conversion(St, isC);

	// deallocate data
	free(data);

	return result;
}
