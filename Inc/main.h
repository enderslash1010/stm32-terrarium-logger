
#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f103xb.h"
#include "delay.h"

inline static void config_gpio(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t mode, uint8_t cnf)
{
	if (pin < 8) // CRL
	{
		GPIOx->CRL &= ~((0b11 << (pin * 4)) | (0b11 << ((pin * 4) + 2))); // Clear MODE(pin) and CNF(pin)
		GPIOx->CRL |= (((mode & 0b11) << (pin * 4)) | ((cnf & 0b11) << ((pin * 4) + 2))); // Set MODE(pin) and CNF(pin)
	}
	else // CRH
	{
		GPIOx->CRH &= ~((0b11 << ((pin - 8) * 4)) | (0b11 << (((pin - 8) * 4) + 2))); // Clear MODE(pin) and CNF(pin)
		GPIOx->CRH |= (((mode & 0b11) << ((pin - 8) * 4)) | ((cnf & 0b11) << (((pin - 8) * 4) + 2))); // Set MODE(pin) and CNF(pin)
	}
}

#endif /* MAIN_H_ */
