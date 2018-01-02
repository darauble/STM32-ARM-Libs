/*
 * Print out strings to the selected U(S)ART of the STM32F10x processors.
 *
 * usart_utils.h
 *
 *  Created on: Oct 12, 2015
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

#ifndef USART_UTILS_H_
#define USART_UTILS_H_

#include <sys/cdefs.h>

#include "stm32f10x.h"

__BEGIN_DECLS


/**
 * Enable USART for serial communication using standard pins.
 * If you want to use alternative pins, do that yourself :-)
 *
 * @param: USART port to be used
 * @param: baud rate
 */
void enable_usart(USART_TypeDef*, uint32_t);

/**
 * TODO: implement disabling of USART.
 */
void disable_usart(USART_TypeDef*);

/**
 * Print text to selected USART.
 * @param: USART port to be used
 * @param: string to be printed
 */
int print_usart(USART_TypeDef*, const char*);

__END_DECLS

#endif /* USART_UTILS_H_ */
