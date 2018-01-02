/*
 * Various macroses to speed up initialization of STM32F10x peripherals
 *
 * makrosai.h
 *
 *  Created on: Feb 15, 2016
 *      Author: Darau, blė
 *
 *  This file is a part of personal use libraries developed specifically for STM32 Cortex-M3
 *  microcontrollers, most notably STM32F103 series.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#ifndef MAKROSAI_H_
#define MAKROSAI_H_

#include "stm32f10x.h"


// AFIO enable/disable
#define AFIO_ENABLE	RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN;
#define AFIO_DISABLE	RCC->APB2ENR &= ~RCC_APB2ENR_AFIOEN;

// GPIOA enable/disable
#define GPIOA_ENABLE	RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;
#define GPIOA_DISABLE	RCC->APB2ENR &= ~RCC_APB2ENR_IOPAEN;

// GPIOB enable/disable
#define GPIOB_ENABLE	RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;
#define GPIOB_DISABLE	RCC->APB2ENR &= ~RCC_APB2ENR_IOPBEN;


// JTAG and SWD manipulation
#define SWJ_CLEAR ((uint32_t)0xF8FFFFFF)
#define JTAG_OFF_SWD_ON  ((uint32_t)0x02000000)
#define JTAG_OFF_SWD_OFF ((uint32_t)0x04000000)

// Disable JTAG, leave SWD for debugging
#define REMAP_SWJ_JTAGDISABLE \
	AFIO_ENABLE; \
	AFIO->MAPR &= SWJ_CLEAR; \
	AFIO->MAPR |= JTAG_OFF_SWD_ON;

// Fully disable hardware debugging (e.g. to use all pins to the maximum)
#define REMAP_JTAG_FULL_DISABLE \
	AFIO_ENABLE; \
	AFIO->MAPR &= SWJ_CLEAR; \
	AFIO->MAPR |= JTAG_OFF_SWD_OFF;

// Chefk if a pin is High
#define PIN_HIGH(REG, PIN) ((REG & PIN) > 0) ? 1 : 0

#endif /* MAKROSAI_H_ */
