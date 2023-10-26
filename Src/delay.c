
#include "delay.h"

static volatile uint32_t delayCount = 0;

void SysTick_Handler(void)
{
	delayCount++;
}

void delay_init(void)
{
	__disable_irq();
	SysTick->LOAD = (uint32_t)((72000000/1000) - 1); // Interrupts every 1ms
	SysTick->VAL = 0; // Initial value of 0
	SysTick->CTRL = 0b110; // Set Clock Source and SysTick Exception Request
	__enable_irq();
}

void delay_ms(uint32_t ms)
{
	delayCount = 0; // Set delayCount to 0
	SysTick->CTRL |= (1 << 0); // Enable SysTick
	while (delayCount < ms);
	SysTick->CTRL &= ~(1 << 0); // Disable SysTick
}
