/*
 * The implementation of the library to read Dallas temperature sensors.
 *
 * LIBRARY IS DEPRECATED AND NOT SUPPORTED ANYMORE.
 *
 * dallas.c
 *
 *  Created on: Dec 2, 2016
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

#include "dallas.h"

void ds_convert_all(owu_struct_t *wire)
{
	owu_reset(wire);
	owu_skip(wire);
	owu_write_byte(wire, CMD_START_CONV);
}

void ds_read_scratchpad(owu_struct_t *wire, uint8_t *addr, uint8_t *scratch_pad)
{
	owu_reset(wire);
	owu_select_device(wire, addr);
	owu_write_byte(wire, CMD_READ_SCR);

	uint8_t i;
	for (i=0; i<__SCR_LENGTH; i++) {
		scratch_pad[i] = owu_read_byte(wire);
	}
}
