
#include "spi_hal.h"

void spi1_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable GPIOA Clock
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable AFIO Clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SPI1 Clock

	// Reset PA4, PA5, and PA7 MODE and CNF to 0b00
	GPIOA->CRL &= ~((GPIO_CRL_MODE4 | GPIO_CRL_CNF4)
				  | (GPIO_CRL_MODE5 | GPIO_CRL_CNF5)
	              | (GPIO_CRL_MODE7 | GPIO_CRL_CNF7));

	// Configure PA4 (NSS): Mode 0b11, CNF 0b00
	GPIOA->CRL |= (0b11 << GPIO_CRL_MODE4_Pos);

	// Configure PA5 (SCK): Mode 0b11, CNF 0b10
	GPIOA->CRL |= (0b11 << GPIO_CRL_MODE5_Pos) | (0b10 << GPIO_CRL_CNF5_Pos);

	// Configure PA7 (MOSI): Mode 0b11, CNF 0b10
	GPIOA->CRL |= (0b11 << GPIO_CRL_MODE7_Pos) | (0b10 << GPIO_CRL_CNF7_Pos);

	// Set PA4 (NSS) high
	GPIOA->ODR |= (1 << 4);

	// Configure SPI registers
	SPI1->CR1 = SPI_CR1_SSM
			| SPI_CR1_SSI
			| SPI_CR1_MSTR
			| SPI_CR1_BR_2;

	// Enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_write(SPI_TypeDef* SPI, uint8_t data, GPIO_TypeDef* GPIOx)
{
	while (!(SPI->SR & SPI_SR_TXE)); // Wait for Tx buffer to be empty
	GPIOx->ODR &= ~(1 << 4); // Set Enable to 0
	SPI->DR = data; // The transmit sequence begins when a byte is written in the Tx Buffer
	while (!(SPI->SR & SPI_SR_TXE)); // Wait for Tx buffer to be empty
	while( SPI->SR & SPI_SR_BSY );  // Wait until SPI not busy
	GPIOx->ODR |= (1 << 4); // Set Enable to 1
}
