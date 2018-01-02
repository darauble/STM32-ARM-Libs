/*
 * The header of the library to read Dallas temperature sensors. It uses
 * OneWire library, which provides core underlying protocol functionality.
 * Intended for use with SMT32 Cortex-M3 microcontrolers.
 *
 * TODO:
 * - implement writing to scratchpad (set alarm threshold, conversion resolution etc.)
 * - implement handling of different sensors
 *
 * dallas.h
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

#ifndef INCLUDE_DALLAS_H_
#define INCLUDE_DALLAS_H_

// Sensor types
#define TYPE_DS18S20 0x10
#define TYPE_DS18B20 0x28
#define TYPE_DS1822  0x22

// Converstion resolution
#define RES_9  0x1F //  9 bit
#define RES_10 0x3F // 10 bit
#define RES_11 0x5F // 11 bit
#define RES_12 0x7F // 12 bit

typedef enum {
	SCR_L = 0,
	SCR_H,
	SCR_HI_ALARM,
	SCR_LO_ALARM,
	SCR_CFG,
	SCR_FFH,
	SCR_RESERVED,
	SCR_10H,
	SCR_CRC,
	__SCR_LENGTH,
} scratch_pad_index;

#define CMD_READ_SCR 0xBE
#define CMD_START_CONV 0x44

#include <sys/cdefs.h>
#include "onewire_usart_dma.h"

__BEGIN_DECLS

/**
 * Issue a global command to start temperature conversion for
 * all connected sensors.
 *
 * @param: OneWire describing and initialized strucutre
 */
void ds_convert_all(owu_struct_t *wire);

/**
 * Read a scratchpad of a particular sensor identified by its serial number
 *
 * @param: OneWire describing and initialized strucutre
 * @param: 8 byte serial of the sensor
 * @param: a pointer to put read scratchpad to
 */
void ds_read_scratchpad(owu_struct_t *wire, uint8_t *addr, uint8_t *scratch_pad);

__END_DECLS

#endif /* INCLUDE_DALLAS_H_ */
