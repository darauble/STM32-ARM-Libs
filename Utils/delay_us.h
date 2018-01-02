/*
 * Utility to add microseconds delays when required.
 * Uses debug registers of the STM32F10x processors to count CPU cycles
 *
 * delay_us.h
 *
 *  Created on: Nov 25, 2016
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

#ifndef INCLUDE_DELAY_US_H_
#define INCLUDE_DELAY_US_H_

#include <sys/cdefs.h>
#include "stm32f10x.h"

// Define to enable IWDG_KR register reload while performing delay
//#define IWDG_RELOAD

__BEGIN_DECLS

/**
 * Init debug registers (DWT) to enable CPU cycles count as a
 * measuring point for µs.
 */
void init_delay_us(void);

/**
 * Perform busy-wait type delay for appropriate count of
 * microseconds.
 *
 * @param: microseconds to delay
 */
void delay_us(uint32_t us);

__END_DECLS

#endif /* INCLUDE_DELAY_US_H_ */
