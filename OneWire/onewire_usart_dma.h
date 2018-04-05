/*
 * The header of the OneWire protocol library for work with SMT32 Cortex-M3 microcontrolers
 *
 * LIBRARY IS DEPRECATED AND NOT SUPPORTED ANYMORE.
 *
 *
 * onewire_usart_dma.h
 *
 *  Created on: Nov 22, 2016
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

#ifndef INCLUDE_ONEWIRE_USART_DMA_H_
#define INCLUDE_ONEWIRE_USART_DMA_H_

#include <sys/cdefs.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"

#define OW_NONE		0
#define OW_OK			1
#define OW_ERR		2
#define OW_NOTHING 	3

// Define only one of the following. Either uncomment or, better, add to the compiler with -D
//#define OW_MODE_BITBANG  // Use bitbanging as OneWire communication - TODO, still under construction
#define OW_MODE_USART  // Use particular U(S)ART interface for OneWire communication
//#define OW_MODE_USART_DMA  // Use USART and DMA for byte sequence writing and reading - TODO, still under construction

// Ucomment if using any USART mode and FreeRTOS. Task will yield,
// while doing hardware based operations.
//#define OW_FREERTOS

// Undefine if OneWire address search is not required, save several bytes
#define OW_LIB_SEARCH

// The structure describing OneWire instance. Don't use it directly, only
// provide required variables and pointer to this structure to appropriate
// owu_init(...) function (constructor).
typedef struct owu_struct {
#ifdef OW_MODE_BITBANG
	GPIO_TypeDef *gpio;
	uint32_t *cr_reg;
	uint32_t input_mask;
	uint32_t output_mask;
#elif defined(OW_MODE_USART) || defined(OW_MODE_USART_DMA)
	USART_TypeDef* USARTx;
	uint16_t brr_9600;
	uint16_t brr_115200;
#endif
#ifdef OW_MODE_USART_DMA
	DMA_TypeDef *dma;
	DMA_Channel_TypeDef *dma_rx;
	DMA_Channel_TypeDef *dma_tx;
	uint32_t dma_rx_flag;
	uint32_t dma_tx_flag;
	uint32_t dma_ifcr_clear;
#endif
#ifdef OW_LIB_SEARCH
	uint8_t last_discrepancy;
	uint8_t last_device_flag;
	uint8_t last_family_discrepancy;
	uint8_t rom_no[8];
#endif
} owu_struct_t;

__BEGIN_DECLS

#ifdef OW_MODE_BITBANG
/**
 * OneWire structure initializer for bitbanging mode
 * @param GPIO to be used
 * @param pin to be used (a number from 0 to 16)
 * @param pointer to owu_struct_t to be initialized
 */
void owu_init(GPIO_TypeDef *, uint16_t, owu_struct_t *);
#elif defined(OW_MODE_USART) || defined(OW_MODE_USART_DMA)
/**
 * OneWire structure initializer for USART mode (including DMA)
 * @param USART to be used
 * @param pointer to owu_struct_t to be initialized
 */
void owu_init(USART_TypeDef *, owu_struct_t *);
#endif

/**
 * Resets the OneWire bus with reset pulse.
 * @param owu_struct_t type initialized instance
 * @return OW_OK if there are connected OneWire devices, OW_NOTHING if bus is empty
 */
uint8_t owu_reset(owu_struct_t *);

/**
 * Writes a byte to OneWire bus using defined mode.
 * @param owu_struct_t type initialized instance
 * @param a byte to be written to the bus
 * @return OW_OK, no check is implemented
 */
uint8_t owu_write_byte(owu_struct_t *, uint8_t);

/**
 * Reads a byte from OneWire bus using defined mode.
 * @param owu_struct_t type initialized instance
 * @return a read byte
 */
uint8_t owu_read_byte(owu_struct_t *);

/**
 * Send skip ROM address check command
 * @param owu_struct_t type initialized instance
 */
void owu_skip(owu_struct_t *);

/**
 * Selects particular device by address for further
 * commands.
 * @param owu_struct_t type initialized instance
 * @param address of the device
 */
void owu_select_device(owu_struct_t *, const uint8_t *);

#ifdef OW_LIB_SEARCH
/**
 * Resets the search, when one is required to be restarted
 * from the beginning (i.e. full search again).
 * @param owu_struct_t type initialized instance
 */
void owu_reset_search(owu_struct_t *wire);

/**
 * OneWire device search iterator, copied from Arduino OneWire library
 * by Jim Studt, Paul Stoffregen and others, downloaded from
 * http://www.pjrc.com/teensy/td_libs_OneWire.html
 *
 * @param owu_struct_t type initialized instance
 * @param pointer to 8 byte array to place found address
 */
uint8_t owu_search(owu_struct_t *wire, uint8_t *addr);
#endif

__END_DECLS

#if (defined(OW_MODE_BITBANG) && defined(OW_MODE_USART)) \
	|| (defined(OW_MODE_BITBANG) && defined(OW_MODE_USART_DMA)) \
	|| (defined(OW_MODE_USART) && defined(OW_MODE_USART_DMA))
#error "Only one One Wire library mode can be defined at one time"
#endif

#endif /* INCLUDE_ONEWIRE_USART_DMA_H_ */
