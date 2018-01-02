/*
 * Utility to add microseconds delays when required.
 * Uses debug registers of the STM32F10x processors to count CPU cycles
 *
 * delay_us.c
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

#include "delay_us.h"

#define DWT_CTRL   (*(uint32_t*) 0xE0001000)
#define DWT_CYCCNT (*(uint32_t*) 0xE0001004)

#ifdef IWDG_RELOAD
#define KR_KEY_Reload ((uint16_t)0xAAAA)
#endif

static uint32_t cycles_us;

void init_delay_us()
{
	cycles_us = SystemCoreClock / 1000000;
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT_CYCCNT = 0;
		DWT_CTRL |= 0x01;
	}
}

void delay_us(uint32_t us)
{
	volatile uint32_t cyc_curr = DWT_CYCCNT;
	volatile uint32_t cyc_total = (cycles_us * us);
	while (DWT_CYCCNT - cyc_curr < cyc_total) {
#ifdef IWDG_RELOAD
		IWDG->KR = KR_KEY_Reload;
#endif
	}
}
