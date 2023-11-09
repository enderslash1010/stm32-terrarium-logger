
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

	// Configure DMA with SPI, since SPI is used to only transmit (in this program), we only need to enable the SPI Tx DMA channel
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable DMA Clock
	DMA1_Channel3->CPAR = (uint32_t) &SPI1->DR; // Set peripheral register address in DMA_CPARx
	DMA1_Channel3->CCR |= DMA_CCR_DIR | DMA_CCR_TCIE; // Configure data transfer direction (memory to peripheral) and interrupts in DMA_CCRx
	SPI1->CR2 |= SPI_CR2_TXDMAEN; // Enable DMA in SPI_CR2

	// Enable NVIC for DMA Interrupts
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	// Enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_write(SPI_TypeDef* SPI, uint8_t data, GPIO_TypeDef* GPIOx)
{
	while (!(SPI->SR & SPI_SR_TXE)); // Wait for Tx buffer to be empty
	GPIOx->ODR &= ~(1 << 4); // Set Enable to 0
	SPI->DR = data; // The transmit sequence begins when a byte is written in the Tx Buffer
	while (!(SPI->SR & SPI_SR_TXE)); // Wait for Tx buffer to be empty
	while (SPI->SR & SPI_SR_BSY);  // Wait until SPI not busy
	GPIOx->ODR |= (1 << 4); // Set Enable to 1
}

volatile uint8_t dma_in_progress = 0; // Mutex used to ensure a DMA transmission currently in progress is not overwritten by the next one
void DMA1_Channel3_IRQHandler(void)
{
	DMA1_Channel3->CCR &= ~DMA_CCR_EN; // Disable DMA Channel
	DMA1->IFCR = DMA_IFCR_CTCIF3; // Clear DMA Global Flag
	dma_in_progress = 0; // Surrender mutex
}

void spi_write_dma(SPI_TypeDef* SPI, uint8_t data, GPIO_TypeDef* GPIOx)
{
	while (dma_in_progress); // Wait until last DMA transfer is complete
	GPIOx->ODR &= ~(1 << 4); // Set Enable to 0

	// Only transmitting, just need to enable the SPI Tx DMA channel
	DMA1_Channel3->CMAR = (uint32_t) &data; // Set memory address in DMA_CMARx
	DMA1_Channel3->CNDTR = 1; // Configure total number of data to be transferred in DMA_CNDTRx (1 byte)
	DMA1_Channel3->CCR |= DMA_CCR_EN; // Activate channel by setting enable bit in DMA_CCRx, this sends data
	dma_in_progress = 1; // Give mutex to DMA1_Channel3 IRQ

	while (dma_in_progress); // Wait for IRQ to give back mutex

	// These spinlocks are required to avoid corrupting the last transmission (rm0008 pg. 719)
	while (!(SPI->SR & SPI_SR_TXE)); // Wait until TxE = 1
	while(SPI->SR & SPI_SR_BSY); // Wait until BSY = 0
	GPIOx->ODR |= (1 << 4); // Set Enable to 1
}
