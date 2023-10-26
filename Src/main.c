
#include "main.h"
#include "pcd8544.h"
#include "sht30.h"

#define RST_PIN 8
#define DC_PIN 9
#define BL_PIN 10
#define Vcc_PIN 11

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

/* Nokia Screen
 * PA8: RST
 * PA4: NSS/CE
 * PA9: DC
 * PA7: MOSI
 * PA5: SCK
 * PA11: Vcc
 * PA10: BL
 */

/* SHT30
 * PB6: SCL
 * PB7: SDA
 */

int main(void)
{
	clock_init(); // Initialize clock to 72 MHz
	delay_init(); // Initialize delay functions

	PCD8544_t screen = pcd8544_init(GPIOA, RST_PIN, DC_PIN, BL_PIN, Vcc_PIN, 0x70, 0, 0b011); // Initialize screen
	pcd8544_toggle_backlight(&screen); // Turn on screen backlight

	SHT30_t sensor1 = sht30_init(I2C1, 0x44); // Initialize sensor 1 on I2C1

	char temperatureStr[8], humidityStr[8]; // Initialize string buffers for temp and humidity, these are the strings sent to the screen
	const char degreeSign[] = {0x80, 0x0}; // Initialize and define degree sign character (0x80 in default font, ending in null terminator)

    while (1)
    {
    	SensorValues_t sensor1Vals = sht30_get_sensor_value(&sensor1, 1, 1, 0); // Get temp and humidity values from sensor 1

    	sprintf(temperatureStr, "%.1f", sensor1Vals.temperature); // Convert temperature from double to string
    	sprintf(humidityStr, "%.0f", sensor1Vals.relative_humidity); // Convert humidiy from double to string

    	/*
    	 * Displays temp and humidity on the screen in form:
    	 *
    	 * Temp.:XX.X°F
    	 * Humidity:XX%
    	 */
    	pcd8544_set_cursor(&screen, 0, 0);
    	pcd8544_write_string(&screen, "Temp.:");
    	pcd8544_write_string(&screen, temperatureStr);
    	pcd8544_write_string(&screen, degreeSign);
    	pcd8544_write_string(&screen, "F");
    	pcd8544_set_cursor(&screen, 0, 1);
    	pcd8544_write_string(&screen, "Humidity:");
    	pcd8544_write_string(&screen, humidityStr);
    	pcd8544_write_string(&screen, "%");

    	delay_ms(1000);
    }
}
