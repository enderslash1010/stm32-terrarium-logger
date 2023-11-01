
#include "i2c_hal.h"

#define DELAY 16000

/*
 * Note: When the STOP, START or PEC bit is set, the software must not perform any write access
 * to I2C_CR1 before this bit is cleared by hardware. Otherwise there is a risk of setting a
 * second STOP, START or PEC request (make sure STOP/START/PEC is cleared before doing any writes to I2C_CR1)
 */

/*
 * Note: Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag, even if the ADDR flag was
 * set after reading I2C_SR1. Consequently, I2C_SR2 must be read only when ADDR is found
 * set in I2C_SR1 or when the STOPF bit is cleared
 */

// Master sends START condition
// Assumes I2C is in slave mode, switches to master mode upon sending START
static I2C_ERROR_CODE i2c_start(I2C_TypeDef* I2C)
{
	uint16_t timeout = 0;

	I2C->CR1 |= I2C_CR1_START; // Sends START condition when the bus is free
	// Then the master waits for a read of the SR1 register followed by a write in the DR register with the Slave address
	while (!(I2C->SR1 & I2C_SR1_SB)) if (timeout++ >= DELAY) return I2C_START_ERROR; // Wait for start condition to be generated
	return I2C_OK;
}

// Master sends START condition, uses I2C with interrupts
static void i2c_start_it(I2C_TypeDef* I2C)
{
	I2C->CR1 |= I2C_CR1_START; // Sends START condition
}

// Master sends STOP condition
// Used by both blocking and interrupt modes
static void i2c_stop(I2C_TypeDef* I2C)
{
	I2C->CR1 |= I2C_CR1_STOP; // Send STOP condition
}

// Master sends address of device it wants to communicate with
// Master enters transmitter (LSB=0) or receiver mode (LSB=1)
static I2C_ERROR_CODE i2c_send_addr(I2C_TypeDef* I2C, uint8_t addr, uint8_t LSB)
{
	uint16_t timeout = 0;

	uint8_t msg = (addr << 1) | (LSB & 0x1);
	I2C->DR = msg; // Write msg to DR, sends addr on SDA
	while (!(I2C->SR1 & I2C_SR1_ADDR)) if (timeout++ >= DELAY) return I2C_ADDR_ERROR; // Wait for addr to be sent by hardware
	if (I2C->SR2); // Read SR2 to clear ADDR (only when ADDR is found to be set)
	return I2C_OK;
}

// Data structures to store info for use with I2C interrupts; These structures should be set before calling i2c_start_it()
volatile uint8_t currAddr[2] = {0, 0};
volatile uint8_t currLSB[2] = {0, 0};
volatile uint8_t* currData[2] = {0, 0};
volatile uint16_t currLen[2] = {0, 0};

// Master sends address of device it wants to communicate with; Only called in an interrupt request routine (IRQ)
// Master enters transmitter (LSB=0) or receiver mode (LSB=1)
static void i2c_send_addr_it(I2C_TypeDef* I2C)
{
	int i = I2C == I2C1 ? 0 : 1;
	uint8_t msg = (currAddr[i] << 1) | (currLSB[i] & 0x1);
	I2C->DR = msg; // Write msg to DR, sends addr on SDA
}

// Sends len bytes of data to previously called device
// Assumes device address is already sent
static I2C_ERROR_CODE i2c_send_data(I2C_TypeDef* I2C, uint8_t* data, uint16_t len)
{
	uint16_t timeout = 0;

	for (int i = 0; i < len; i++) // Send all len # of data bytes
	{
		while (!(I2C->SR1 & I2C_SR1_TXE)); // Wait for the transmission buffer (TX) to be empty
		I2C->DR = data[i]; // Write data to DR, sends data on SDA
	}
	while (!(I2C->SR1 & I2C_SR1_TXE) && !(I2C->SR1 & I2C_SR1_BTF)) if (timeout++ >= DELAY) return I2C_DATA_ERROR; // Wait for TX to be empty, and byte transfer to finish (BTF=1)
	return I2C_OK;
}

// Sends data from previously set currData to previously called device; Only called in an interrupt request routine (IRQ)
static void i2c_send_data_it(I2C_TypeDef* I2C)
{
	uint16_t len = I2C == I2C1 ? currLen[0] : currLen[1];
	volatile uint8_t* data = I2C == I2C1 ? currData[0] : currData[1];

	for (int i = 0; i < len; i++) // Send all len # of data bytes
	{
		while (!(I2C->SR1 & I2C_SR1_TXE)); // Wait for the transmission buffer (TX) to be empty
		I2C->DR = data[i]; // Write data to DR, sends data on SDA
	}

	while (!(I2C->SR1 & I2C_SR1_TXE)); // Wait for the transmission buffer (TX) to be empty
	i2c_stop(I2C);
	if (I2C == I2C1) NVIC_DisableIRQ(I2C1_EV_IRQn);
	else if (I2C == I2C2) NVIC_DisableIRQ(I2C2_EV_IRQn);
}

// Returns the byte in the receive buffer (Rx), and responds with ACK (ack=1) or NACK (ack=0)
static uint8_t i2c_get_data(I2C_TypeDef* I2C, uint8_t ack)
{
	if (ack) I2C->CR1 |= I2C_CR1_ACK; // Enables sending of an ACK when reading from the data register (I2C->DR)
	else I2C->CR1 &= ~(I2C_CR1_ACK); // Sends NACK when reading from data register

	while (!(I2C->SR1 & I2C_SR1_RXNE)); // Wait for receive buffer to be not empty
	uint8_t data = I2C->DR; // Get byte from receive buffer (by reading from data register)
	return data;
}

// Puts retrieved data from device into the currData structure to be returned from i2c_read_it() later
static void i2c_get_data_it(I2C_TypeDef* I2C)
{
	uint16_t len = I2C == I2C1 ? currLen[0] : currLen[1];
	volatile uint8_t* data = I2C == I2C1 ? currData[0] : currData[1];

	int i = 0;
	for (; i < len - 1; i++)
	{
		while (!(I2C->SR1 & I2C_SR1_RXNE)); // Wait for receive buffer to be not empty
		data[i] = I2C->DR; // Get byte from receive buffer (by reading from data register)
	}

	I2C->CR1 &= ~(I2C_CR1_ACK); // Sends NACK when reading from data register
	while (!(I2C->SR1 & I2C_SR1_RXNE)); // Wait for receive buffer to be not empty
	data[i] = I2C->DR; // Get byte from receive buffer (by reading from data register)

	i2c_stop(I2C);
	if (I2C == I2C1) NVIC_DisableIRQ(I2C1_EV_IRQn);
	else if (I2C == I2C2) NVIC_DisableIRQ(I2C2_EV_IRQn);
}

// Sends a data byte through the I2C peripheral specified
I2C_ERROR_CODE i2c_write(I2C_TypeDef* I2C, uint8_t addr, uint8_t* data, uint8_t len)
{
	while (I2C->SR2 & I2C_SR2_BUSY); // Wait for I2C bus to be not busy
	if (i2c_start(I2C) != I2C_OK)
	{
		i2c_stop(I2C);
		return I2C_START_ERROR;
	}
	if (i2c_send_addr(I2C, addr, 0) != I2C_OK)
	{
		i2c_stop(I2C);
		return I2C_ADDR_ERROR;
	}
	if (i2c_send_data(I2C, data, len) != I2C_OK)
	{
		i2c_stop(I2C);
		return I2C_DATA_ERROR;
	}
	i2c_stop(I2C);
	return I2C_OK;
}

// Reads numBytes bytes from the device
// Returns a numBytes length array with the data
uint8_t* i2c_read(I2C_TypeDef* I2C, uint8_t addr, uint8_t numBytes)
{
	while (I2C->SR2 & I2C_SR2_BUSY); // Wait for I2C bus to be not busy
	if (i2c_start(I2C) != I2C_OK) // Send START condition
	{
		i2c_stop(I2C);
		return 0;
	}
	if (i2c_send_addr(I2C, addr, 1) != I2C_OK) // Send device address, and receive ACK from device
	{
		i2c_stop(I2C);
		return 0;
	}

	uint8_t* data = (uint8_t*) malloc(numBytes * sizeof(uint8_t)); // This data pointer is freed in sht30 library
	int i = 0;
	for (; i < numBytes - 1; i++) // Get data, and reply with ACK
	{
		data[i] = i2c_get_data(I2C, 1);
	}

	// Get last byte of data, and replay with NACK
	data[i] = i2c_get_data(I2C, 0);

	i2c_stop(I2C);
	return data;
}

// Function called from I2Cx_EV_IRQHandler
static void I2C_EV(I2C_TypeDef* I2C)
{
	if (I2C->SR1 & I2C_SR1_SB) // SB is set
	{
		i2c_send_addr_it(I2C);
	}
	else if (I2C->SR1 & I2C_SR1_ADDR) // ADDR is set
	{
		if (I2C->SR2); // Read SR2 to clear ADDR (only when ADDR is found to be set)

		int i = I2C == I2C1 ? 0 : 1;
		if (currLSB[i]) i2c_get_data_it(I2C); // Read
		else i2c_send_data_it(I2C); // Write
	}
	// ADD10 not handled
}

// IRQ for I2Cx Events (SB, ADDR, ADD10, STOPF, BTF)
void I2C1_EV_IRQHandler(void) { I2C_EV(I2C1); }
void I2C2_EV_IRQHandler(void) { I2C_EV(I2C2); }

// Does the same thing as i2c_write(), but uses interrupts instead of blocking
void i2c_write_it(I2C_TypeDef* I2C, uint8_t addr, uint8_t* data, uint8_t len)
{
	while (I2C->SR2 & I2C_SR2_BUSY); // Wait for I2C bus to be not busy

	if (I2C == I2C1) NVIC_EnableIRQ(I2C1_EV_IRQn);
	else if (I2C == I2C2) NVIC_EnableIRQ(I2C2_EV_IRQn);

	int i = I2C == I2C1 ? 0 : 1;
	currLen[i] = len;
	currData[i] = data;
	currAddr[i] = addr;
	currLSB[i] = 0;

	i2c_start_it(I2C); // START condition kicks off all communication, interrupts handle the rest
}

// Does the same thing as i2c_read(), but uses interrupts instead of blocking
uint8_t* i2c_read_it(I2C_TypeDef* I2C, uint8_t addr, uint8_t numBytes)
{
	while (I2C->SR2 & I2C_SR2_BUSY); // Wait for I2C bus to be not busy

	int i = 0;
	if (I2C == I2C1) NVIC_EnableIRQ(I2C1_EV_IRQn);
	else if (I2C == I2C2) NVIC_EnableIRQ(I2C2_EV_IRQn);

	// Write to i2c_get_data_it vars
	uint8_t* data = (uint8_t*) malloc(numBytes * sizeof(uint8_t));
	currData[i] = data;
	currLen[i] = numBytes;

	// Write to i2c_send_addr_it vars
	currAddr[i] = addr;
	currLSB[i] = 1;

	I2C->CR1 |= I2C_CR1_ACK; // Enables sending of an ACK when reading from the data register (I2C->DR)

	i2c_start_it(I2C);
	while (I2C->SR2 & I2C_SR2_BUSY); // Wait for I2C bus to be not busy
	return data;
}

// Initializes I2Cx with 100kHz speed
void i2c_init(I2C_TypeDef* I2C, uint32_t pclk1)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // Enable GPIOB Clock
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable AFIO Clock

	if (I2C == I2C1)
	{
		// Configure GPIO for I2C1
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // Enable I2C1 Clock

		// Uses default AFIO mapping for I2C1 (PB6->SCL, PB7->SDA), no need for remapping
		I2C->CR1 &= ~(I2C_CR1_PE);
		GPIOB->CRL |= (0b11 << 24) | (0b11 << 28); // Set PB6 and PB7 to output mode
		GPIOB->ODR |= (1 << 6) | (1 << 7); // From errata, set ODR to 1
		while (!(((GPIOB->IDR & (1 << 6))) && (GPIOB->IDR & (1 << 7)))); // From errata, check IDR for PB6 and PB7 are high
		GPIOB->ODR &= ~(1 << 7); // From errata, set SCL ODR to 0
		while (GPIOB->IDR & (1 << 7)); // From errata, check if SCL is 0
		GPIOB->ODR &= ~(1 << 6); // From errata, set SDA to 0
		while (GPIOB->IDR & (1 << 6)); // From errata, check if SDA is 0
		GPIOB->ODR |= (1 << 7); // From errata, set SCL ODR to 1
		while (!(GPIOB->IDR & (1 << 7))); // From errata, check if SCL is 1
		GPIOB->ODR |= (1 << 6); // From errata, set SDA ODR to 1
		while (!(GPIOB->IDR & (1 << 6))); // From errata, check if SDA is 1

		GPIOB->CRL |= (0b11 << 26) | (0b11 << 30); // Set PB6 and PB7 to alternate function output open-drain
	}
	else // I2C2
	{
		// Configure GPIO for I2C2
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // Enable I2C2 Clock
		// Uses default AFIO mapping for I2C2 (PB10->SCL, PB11->SDA), no need for remapping
		I2C->CR1 &= ~(I2C_CR1_PE); // Make sure I2C peripheral is disabled
		GPIOB->CRH |= (0b11 << 8) | (0b11 << 12); // Set PB10 and PB11 to output mode
		GPIOB->ODR |= (1 << 10) | (1 << 11); // From errata, set ODR to 1
		while (!(((GPIOB->IDR & (1 << 10))) && (GPIOB->IDR & (1 << 11)))); // From errata, check IDR for PB10 and PB11 are high
		GPIOB->ODR &= ~(1 << 10); // From errata, set SCL ODR to 0
		while (GPIOB->IDR & (1 << 10)); // From errata, check if SCL is 0
		GPIOB->ODR &= ~(1 << 11); // From errata, set SDA to 0
		while (GPIOB->IDR & (1 << 11)); // From errata, check if SDA is 0
		GPIOB->ODR |= (1 << 10); // From errata, set SCL ODR to 1
		while (!(GPIOB->IDR & (1 << 10))); // From errata, check if SCL is 1
		GPIOB->ODR |= (1 << 11); // From errata, set SDA ODR to 1
		while (!(GPIOB->IDR & (1 << 11))); // From errata, check if SDA is 1

		GPIOB->CRH |= (0b11 << 10) | (0b11 << 14); // Set PB10 and PB11 to alternate function output open-drain
	}

	// Reset I2C
	I2C->CR1 |= I2C_CR1_SWRST;
	I2C->CR1 &= ~(I2C_CR1_SWRST);

	I2C->OAR1 |= (1 << 14); // I2C_OAR1 bit 14 "Should always be kept at 1 by software"
	I2C->CR2 |= I2C_CR2_ITEVTEN; // Enable interrupts for events

	/*
	 * Example CCR Calculation with PCLK1 = 36MHz
	 *
	 * The I2C clock should be set to standard mode (100kHz), so the period of SCL should be 10us.
	 * T(SCL) = T(high) + T(low) -> 10us = 2*T(high), using a duty cycle of 1:1 (T(high)=T(low)) -> T(high) = 5us
	 * The datasheet provides a formula for CCR: T(high) = CCR * T(PCLK1). We have PCLK1 = 36MHz, so T(PCLK1) = 1/36MHz = 0.0278us
	 * Because we know T(high), we can solve for CCR: 5us = CCR * 0.0278us -> CCR = 5us/0.0278us = 180 (0xB4)
	 */

	uint8_t pclkInMHz = pclk1/1000000; // Stores pclk in MHz (36000000Hz -> 36MHz)
	double Tpclk = 1.0/pclkInMHz; // Stores period of pclk in microseconds (us)

	I2C->CR2 |= pclkInMHz; // Program the peripheral input clock in I2C_CR2 register to pclk MHz in order to generate correct timings
	I2C->CCR |= ((uint16_t)(5/Tpclk)) & 0xFFF; // CCR = 5us/Tpclk(us), cast to uint16_t and masked to 12 LSB
	I2C->TRISE = ((uint8_t)((1000.0/(Tpclk * 1000)) + 1)) & 0x3F; // Configure the rise time register; The datasheet gives formula (maximum SCL rise time)/(T(PCLK1)) + 1 -> 1000ns/T(PCLK1) + 1, since max rise time in standard mode is 1000ns
	I2C->CR1 |= I2C_CR1_PE; // Enable the peripheral in CR1

	if (I2C->SR2 & I2C_SR2_BUSY) i2c_stop(I2C);
}
