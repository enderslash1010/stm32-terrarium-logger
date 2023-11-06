
#include "main.h"
#include "pcd8544.h"
#include "sht30.h"
#include "i2c_hal.h"

#define RST_PIN 8
#define DC_PIN 9
#define BL_PIN 10
#define Vcc_PIN 11

PCD8544_t screen;
volatile uint8_t displayOn = 0;

// Initiates clock to 72 MHz
void clock_init(void)
{
	FLASH->ACR |= FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE; // Sets up flash latency for 48 MHz < SYSCLK < 72 MHz, and enables FLASH prefetch
	RCC->CFGR |= RCC_CFGR_PPRE1_2; // HCLK divided by 2 for APB1
	RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE; // HSE is not divided for PLL (this line doesn't do anything)

	RCC->CR |= RCC_CR_HSEON; // Switch on HSE oscillator
	while (!(RCC->CR & RCC_CR_HSERDY)); // Wait for HSE to be ready

	RCC->CFGR |= RCC_CFGR_PLLSRC; // Set HSE as source clock for PLL
	RCC->CFGR |= RCC_CFGR_PLLMULL9; // Multiply PLL by 9
	RCC->CR |= RCC_CR_PLLON; // Turn on PLL
	while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL to be ready

	RCC->CFGR |= RCC_CFGR_SW_PLL; // Set PLL as main clock source
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)); // Wait for clock source to be set
}

// Set up parts of the screen that don't change
void display_init(void)
{
	static const char degreeSign[] = {0x80, 0x0}; // Degree sign character (0x80 in default font, ending in null terminator)
	static const char plusMinusSign[] = {0x81, 0x0}; // Plus-minus sign character (0x81)

	pcd8544_set_cursor(&screen, 0, 0);
	pcd8544_write_string(&screen, "Terrarium 1:");
	pcd8544_set_cursor_string(&screen, 4, 1);
	pcd8544_write_string(&screen, degreeSign);
	pcd8544_write_string(&screen, "F ");
	pcd8544_set_cursor_string(&screen, 9, 1);
	pcd8544_write_string(&screen, plusMinusSign);
	pcd8544_write_string(&screen, "2%");

	pcd8544_set_cursor(&screen, 0, 3);
	pcd8544_write_string(&screen, "Terrarium 2:");
	pcd8544_set_cursor_string(&screen, 4, 4);
	pcd8544_write_string(&screen, degreeSign);
	pcd8544_write_string(&screen, "F ");
	pcd8544_set_cursor_string(&screen, 9, 4);
	pcd8544_write_string(&screen, plusMinusSign);
	pcd8544_write_string(&screen, "2%");
}

// Initiates button on PB0
void button_init(void)
{
	config_gpio(GPIOB, 0, 0b0, 0b10); // Configure GPIO to be input with pull-up/pull-down
	GPIOB->ODR |= (1 << 0); // Enable pull-up resistor on button pin

	// Configure EXTI Interrupt
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable AFIO clock
	AFIO->EXTICR[0] |= (0b0001 << 0); // Configure AFIO_EXTICR1; Associate EXTI0 with PB0
	EXTI->IMR |= EXTI_IMR_MR0; // Set interrupt mask register EXTI_IMR
	EXTI->FTSR |= EXTI_FTSR_TR0; // Configure interrupt on falling-trigger (when the button goes from un-pressed->pressed)
	NVIC->ISER[0] = (1 << 6); // Enable EXTI0 in NVIC; EXTI0 corresponds to interrupt 6, so the enable bit is in ISER[0] at bit 6
}

// Initializes Timer 2 (TIM2), used to determine when to put display to 'sleep' (Backlight off, display blank)
void timer2_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock

	/*
	 * Calculating Prescaler and Auto-reload Register
	 *
	 * We want T(TIM2) = 10 seconds, so we need to choose ARR and PSC in the range [0, 65535] so that (ARR * PSC)/72000000 = 10 seconds -> ARR * PSC = 720000000
	 * We can choose PSC = 36000, so ARR * 36000 = 720000000 -> ARR = 20000. Therefore, (PSC, ARR) = (36000, 20000)
	 *
	 * The value placed in both of these registers (TIM2_PSC and TIM2_ARR) is subtracted by 1
	 */

	TIM2->PSC = 36000 - 1; // Set prescaler register
	TIM2->ARR = 20000 - 1; // Set auto-reload register

	TIM2->EGR = TIM_EGR_UG; // Generate Update to finish setting prescaler
	while (!(TIM2->SR & TIM_SR_UIF)); // Wait for Update Interrupt Flag (UIF) to be set in status register
	TIM2->SR = ~TIM_SR_UIF; // Clear UIF

	TIM2->DIER |= TIM_DIER_UIE; // Enable Interrupts for TIM2 in the peripheral registers
	NVIC->ISER[0] = (1 << 28); // Enable Interrupts for TIM2 in NVIC; TIM2 uses interrupt 28, so the enable bit is in ISER[0] at bit 28
}

// EXTI0 Interrupt Handler, for wake button to turn on display
void EXTI0_IRQHandler(void)
{
	if (displayOn == 0) // Wake display if it's off
	{
		pcd8544_set_display_control(&screen, 1, 0); // Screen in normal mode
		pcd8544_toggle_backlight(&screen); // Turn on backlight
		display_init();
		displayOn = 1;

		// Set timer for how long to leave screen on, when timer ends turn the display back off
		TIM2->CNT = 0; // Make sure count is at 0
		TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2 counter
	}
	else // If display is already on, then reset counter to 0
	{
		TIM2->CNT = 0;
	}
	EXTI->PR = EXTI_PR_PR0; // clear interrupt flag in EXTI_PR by writing a 1 into PR13
}

// TIM2 Interrupt Handler, to put display to sleep
void TIM2_IRQHandler(void)
{
	TIM2->CR1 &= ~TIM_CR1_CEN; // Disable TIM2 counter

	// Put display to sleep
	pcd8544_set_display_control(&screen, 0, 0); // Set display blank
	pcd8544_toggle_backlight(&screen); // Turn off backlight
	displayOn = 0;

	TIM2->SR = ~TIM_SR_UIF; // Clear Update Interrupt Flag by writing 0 (rc_w0)
}

/*
 * Nokia Screen
 * PA8: RST
 * PA4: NSS/CE
 * PA9: DC
 * PA7: MOSI
 * PA5: SCK
 * PA11: Vcc
 * PA10: BL
 */

/*
 * Wake Button
 * PB8: GPIO Input
 */

/*
 * SHT30 for sensor 1 (I2C1)
 * PB6: SCL
 * PB7: SDA
 */

/*
 * SHT30 for sensor 2 (I2C2)
 * PB10: SCL
 * PB11: SDA
 */

int main(void)
{
	clock_init(); // Initialize clock to 72 MHz
	delay_init(); // Initialize delay functions
	timer2_init(); // Initialize timer 2

	screen = pcd8544_init(GPIOA, RST_PIN, DC_PIN, BL_PIN, Vcc_PIN, 0x70, 0, 0b011); // Initialize screen
	pcd8544_toggle_backlight(&screen); // Turn on screen backlight
	displayOn = 1;

	SHT30_t sensor1 = sht30_init(I2C1, 36000000, 0x44); // Initialize sensor 1 on I2C1
	SHT30_t sensor2 = sht30_init(I2C2, 36000000, 0x44); // Initialize sensor 2 on I2C2

	// Initialize wake button
	button_init();

	uint8_t sensor1Data[6], sensor2Data[6];

	char temperature1Str[8], humidity1Str[8]; // Initialize string buffers for temp and humidity from sensor 1
	char temperature2Str[8], humidity2Str[8]; // Initialize string buffers for temp and humidity from sensor 2

	// Set up parts of the screen that don't change upon startup
	display_init();

	// Start TIM2 to turn off the display after 10 sec, since the display is already on upon start
	TIM2->CR1 |= TIM_CR1_CEN;

    while (1)
    {
    	if (displayOn) // Update display if on
    	{
        	sht30_get_raw_sensor(&sensor1, 1, 1, sensor1Data); // Get temp and humidity values from sensor 1
        	sht30_get_raw_sensor(&sensor2, 1, 1, sensor2Data); // Get temp and humidity values from sensor 2

        	// Wait for both I2C buses to be free
        	while ((I2C1->SR2 & I2C_SR2_BUSY) || (I2C2->SR2 & I2C_SR2_BUSY));

        	// Convert raw sensor data
        	SensorValues_t sensor1Vals = sht30_convert_sensor(sensor1Data, 0);
        	SensorValues_t sensor2Vals = sht30_convert_sensor(sensor2Data, 0);

        	sprintf(temperature1Str, "%.1f", sensor1Vals.temperature); // Convert temperature 1 from double to string
        	sprintf(humidity1Str, "%.0f", sensor1Vals.relative_humidity); // Convert humidiy 1 from double to string

        	sprintf(temperature2Str, "%.1f", sensor2Vals.temperature); // Convert temperature 2 from double to string
        	sprintf(humidity2Str, "%.0f", sensor2Vals.relative_humidity); // Convert humidiy 2 from double to string

        	/*
        	 * Displays temp and humidity on the screen in form:
        	 *
        	 * Terrarium 1:
        	 * XX.X°F XX%
        	 *
        	 * Terrarium 2:
        	 * XX.X°F XX%
        	 */
        	pcd8544_set_cursor(&screen, 0, 1);
        	pcd8544_write_string(&screen, temperature1Str);
        	pcd8544_set_cursor_string(&screen, 7, 1);
        	pcd8544_write_string(&screen, humidity1Str);

        	pcd8544_set_cursor(&screen, 0, 4);
        	pcd8544_write_string(&screen, temperature2Str);
        	pcd8544_set_cursor_string(&screen, 7, 4);
        	pcd8544_write_string(&screen, humidity2Str);

        	delay_ms(1000);
    	}
    }
}
