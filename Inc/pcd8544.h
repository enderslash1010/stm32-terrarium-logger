
#ifndef INC_PCD8544_H_
#define INC_PCD8544_H_

#include "main.h"
#include "spi_hal.h"

typedef struct pcd8544
{
	GPIO_TypeDef* GPIOx;
	uint8_t RST_Pin;
	uint8_t DC_Pin;
	uint8_t BL_Pin;
	uint8_t Vcc_Pin;
} PCD8544_t;

// Software implemented hardware instructions
void pcd8544_function_set(PCD8544_t* screen, uint8_t PD, uint8_t V, uint8_t H);
void pcd8544_write_data(PCD8544_t* screen, uint8_t data);

void pcd8544_set_display_control(PCD8544_t* screen, uint8_t D, uint8_t E);
void pcd8544_set_Y_RAM(PCD8544_t* screen, uint8_t Y);
void pcd8544_set_X_RAM(PCD8544_t* screen, uint8_t X);
void pcd8544_set_temperature_control(PCD8544_t* screen, uint8_t TC);
void pcd8544_set_bias(PCD8544_t* screen, uint8_t BS);
void pcd8544_set_Vop(PCD8544_t* screen, uint8_t Vop);

// Init function
PCD8544_t pcd8544_init(GPIO_TypeDef* GPIOx, uint8_t RST_Pin, uint8_t DC_Pin, uint8_t BL_Pin, uint8_t Vcc_Pin, uint8_t Vop, uint8_t TC, uint8_t BS);

// Misc. control functions
void pcd8544_toggle_backlight(PCD8544_t* screen);
void pcd8544_write_string(PCD8544_t* screen, const char* str);
void pcd8544_set_cursor(PCD8544_t* screen, uint8_t X, uint8_t Y);
void pcd8544_write_bitmap(PCD8544_t* screen, const uint8_t img[504]);

#endif /* INC_PCD8544_H_ */
