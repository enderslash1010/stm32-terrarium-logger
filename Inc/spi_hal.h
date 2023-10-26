
#ifndef SPI_HAL_H_
#define SPI_HAL_H_

#include "main.h"

void spi1_init(void);
void spi_write(SPI_TypeDef* SPI, uint8_t data, GPIO_TypeDef* GPIOx);

#endif /* SPI_HAL_H_ */
