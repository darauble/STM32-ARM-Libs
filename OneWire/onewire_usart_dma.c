/*
 * The source of the OneWire protocol library for work with SMT32 Cortex-M3 microcontrolers
 *
 * LIBRARY IS DEPRECATED AND NOT SUPPORTED ANYMORE.
 *
 * onewire_usart_dma.c
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

#define KR_KEY_Reload	 ((uint16_t)0xAAAA)
#define KR_KEY_Enable	 ((uint16_t)0xCCCC)

// USART1 uses faster clock (APB2) than USART2/3, hence the difference in constans
#define U1_BAUD_9600	((uint16_t)0x1D4C)
#define U1_BAUD_115200 ((uint16_t)0x0271)

#define Ux_BAUD_9600	((uint16_t)0x0EA6)
#define Ux_BAUD_115200 ((uint16_t)0x0138)

#define CR1_CLEAR_Mask				((uint16_t)0xE9F3)  /*!< USART CR1 Mask */
#define CR2_STOP_CLEAR_Mask		 ((uint16_t)0xCFFF)  /*!< USART CR2 STOP Bits Mask */
#define CR3_CLEAR_Mask				((uint16_t)0xFCFF)  /*!< USART CR3 Mask */

#define RESET_BYTE (uint8_t)0xF0

#define SEND_0	0x00
#define SEND_1	0xff
#define OW_R_1	0xff

#include "onewire_usart_dma.h"
#include <string.h>

const uint8_t ow_bits[] = { SEND_0, SEND_1 };

#ifdef OW_MODE_USART_DMA
// TODO: DMA Mode is still under construction
static uint8_t dma_buf[8];
#endif

#ifdef OW_MODE_BITBANG
// TODO: Bitbanging Mode is still under construction
void owu_init(GPIO_TypeDef *gpio, uint16_t pin, owu_struct_t *wire)
{

}

#elif defined(OW_MODE_USART) || defined(OW_MODE_USART_DMA)

void owu_init(USART_TypeDef *USARTx, owu_struct_t *wire)
{
	// Enable alternative I/O
	RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN;
	// Enable appropriate USART clock
	wire->USARTx = USARTx;

	if (wire->USARTx == USART1) {
		RCC->APB2ENR |= RCC_APB2RSTR_USART1RST;
		wire->brr_9600 = U1_BAUD_9600;
		wire->brr_115200 = U1_BAUD_115200;
#ifdef OW_MODE_USART_DMA
		wire->dma = DMA1;
		wire->dma_rx = DMA1_Channel5;
		wire->dma_tx = DMA1_Channel4;
		wire->dma_rx_flag = DMA1_FLAG_TC5;
		wire->dma_rx_flag = DMA1_FLAG_TC4;
		wire->dma_ifcr_clear = ((uint32_t)
				(DMA_ISR_GIF4 | DMA_ISR_TCIF4 | DMA_ISR_HTIF4 | DMA_ISR_TEIF4)
				|(DMA_ISR_GIF5 | DMA_ISR_TCIF5 | DMA_ISR_HTIF5 | DMA_ISR_TEIF5));
#endif
	} else {
		wire->brr_9600 = Ux_BAUD_9600;
		wire->brr_115200 = Ux_BAUD_115200;
		if (wire->USARTx == USART2) {
			RCC->APB1ENR |= RCC_APB1RSTR_USART2RST;
#ifdef OW_MODE_USART_DMA
			wire->dma = DMA1;
			wire->dma_rx = DMA1_Channel6;
			wire->dma_tx = DMA1_Channel7;
			wire->dma_rx_flag = DMA1_FLAG_TC6;
			wire->dma_tx_flag = DMA1_FLAG_TC7;
			wire->dma_ifcr_clear = ((uint32_t)
					(DMA_ISR_GIF6 | DMA_ISR_TCIF6 | DMA_ISR_HTIF6 | DMA_ISR_TEIF6)
					|(DMA_ISR_GIF7 | DMA_ISR_TCIF7 | DMA_ISR_HTIF7 | DMA_ISR_TEIF7));
#endif
		} else if (wire->USARTx == USART3) {
			RCC->APB1ENR |= RCC_APB1RSTR_USART3RST;
#ifdef OW_MODE_USART_DMA
			wire->dma = DMA1;
			wire->dma_rx = DMA1_Channel3;
			wire->dma_tx = DMA1_Channel2;
			wire->dma_rx_flag = DMA1_FLAG_TC3;
			wire->dma_tx_flag = DMA1_FLAG_TC2;
			wire->dma_ifcr_clear = ((uint32_t)
					(DMA_ISR_GIF2 | DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_TEIF2)
					|(DMA_ISR_GIF3 | DMA_ISR_TCIF3 | DMA_ISR_HTIF3 | DMA_ISR_TEIF3));
#endif
		}
	}

	// Enable usart
	wire->USARTx->CR1 |= USART_CR1_UE;

	wire->USARTx->CR2 &= CR2_STOP_CLEAR_Mask; // Clear CR2 stop bits, leave as is for Stop Bits 1
	wire->USARTx->CR1 &= CR1_CLEAR_Mask; // Clear CR1 config
	wire->USARTx->CR1 |= USART_Mode_Rx | USART_Mode_Tx; // 8b word, no parity, RX/TX
	wire->USARTx->CR3 &= CR3_CLEAR_Mask; // Clear CR3 config, leave as is, no hw control
	wire->USARTx->BRR = wire->brr_115200;

	// Enable appropriate GPIO
	if (wire->USARTx == USART1 || wire->USARTx == USART2 ) {
		// Enable GPIOA
		RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;
		if (wire->USARTx == USART1) {
			GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 | GPIO_CRH_CNF10 | GPIO_CRH_MODE10); // Clear configuration
			GPIOA->CRH |= GPIO_CRH_CNF9 | GPIO_CRH_MODE9 | GPIO_CRH_CNF10_0; // Alternative open drain, 50 MHz; in floating
		} else {
			GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2 | GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
			GPIOA->CRL |= GPIO_CRL_CNF2 | GPIO_CRL_MODE2 | GPIO_CRL_CNF3_0;
		}
	}
#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
	else if (wire->USARTx == USART3) {
		// Enable GPIOB
		RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;
		GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
		GPIOB->CRH |= GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11_0;
	}
#endif

#ifdef OW_MODE_USART_DMA
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// Enable DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	// Initialize DMA
	DMA_InitTypeDef DMA_InitStructure;

	// DMA for reading
	DMA_DeInit(wire->dma_rx);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(wire->USARTx->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) dma_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 8;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(wire->dma_rx, &DMA_InitStructure);//*/

	// DMA for writing
	DMA_DeInit(wire->dma_tx);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(wire->USARTx->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) dma_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 8;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(wire->dma_tx, &DMA_InitStructure);
#endif /* OW_MODE_USART_DMA */
#ifdef OW_LIB_SEARCH
	// Initialize search variables
	owu_reset_search(wire);
#endif
}
#endif /* defined(OW_MODE_USART) || defined(OW_MODE_USART_DMA) */

uint8_t owu_reset(owu_struct_t *wire)
{
	uint8_t ow_presence;
	// Set 9600 baud for 480 uS reset pulse
	wire->USARTx->BRR = wire->brr_9600;
	wire->USARTx->SR = ~( USART_FLAG_TC | USART_FLAG_RXNE);
	wire->USARTx->DR = RESET_BYTE;

	while(!(wire->USARTx->SR & USART_FLAG_TC) && !(wire->USARTx->SR & USART_FLAG_RXNE)) {
#ifdef OW_FREERTOS
		taskYIELD();
#endif
	}

	ow_presence = (uint8_t)(wire->USARTx->DR & (uint16_t)0x01FF);
	// Restore "normal" operating speed
	wire->USARTx->BRR = wire->brr_115200;

	if (ow_presence != RESET_BYTE) {
		return OW_OK;
	}

	return OW_NOTHING;
}

uint8_t owu_write_bit(owu_struct_t *wire, uint8_t bit)
{
	wire->USARTx->SR = ~( USART_FLAG_TC | USART_FLAG_RXNE);
	wire->USARTx->DR = ow_bits[bit];

	while(!(wire->USARTx->SR & USART_FLAG_TC) && !(wire->USARTx->SR & USART_FLAG_RXNE)) {
#ifdef OW_FREERTOS
		taskYIELD();
#endif
	}

	return OW_OK;
}

uint8_t owu_read_bit(owu_struct_t *wire)
{
	uint8_t bit;

	wire->USARTx->SR = ~( USART_FLAG_TC | USART_FLAG_RXNE);
	wire->USARTx->DR = OW_R_1;
	while(!(wire->USARTx->SR & USART_FLAG_TC) && !(wire->USARTx->SR & USART_FLAG_RXNE)) {
#ifdef OW_FREERTOS
		taskYIELD();
#endif
	}
	bit = (uint8_t)(wire->USARTx->DR & (uint16_t)0x01FF);
	if (bit == OW_R_1) {
		return 1;
	} else {
		return 0;
	}
}

#ifndef OW_MODE_USART_DMA
uint8_t owu_write_byte(owu_struct_t *wire, uint8_t byte)
{
	uint8_t i;
	for (i=0; i<8; i++) {
		owu_write_bit(wire, byte & 0x01);
		byte >>= 1;
	}
	return OW_OK;
}
#endif
uint8_t owu_read_byte(owu_struct_t *wire)
{
	uint8_t byte = 0, bit, i;
	for (i=0; i<8; i++){
		byte >>= 1;
		bit = owu_read_bit(wire);
		if (bit) {
			byte |= 0x80;
		}
	}
	return byte;
}


#ifdef OW_MODE_USART_DMA
uint8_t owu_write_byte(owu_struct_t *wire, uint8_t byte)
{
	uint8_t i;
	for (i=0; i<8; i++) {
		dma_buf[i] = ow_bits[byte & 0x01];
		byte >>= 1;
	}

	//wire->USARTx->SR = (uint16_t) ~(USART_FLAG_TC); // Clear flags
	wire->dma_tx->CMAR = (uint32_t) dma_buf;
	wire->dma_tx->CNDTR = 8;
	wire->dma->IFCR |= wire->dma_ifcr_clear;
	wire->USARTx->CR3 |= USART_DMAReq_Tx ; // Enable RX/TX DMA
	wire->dma_tx->CCR |= DMA_CCR1_EN;

	while (!(wire->dma->ISR & wire->dma_tx_flag)) {
#ifdef OW_FREERTOS
		taskYIELD();
#endif
	}

	wire->dma_tx->CCR &= ~DMA_CCR1_EN;
	wire->USARTx->CR3 &= (uint16_t) ~USART_DMAReq_Tx;
	//USART_ClearFlag(OW_USART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
	return OW_OK;
}
/*uint8_t owu_write_byte(owu_struct_t *wire, uint8_t byte)
{
	uint8_t i;
	for (i=0; i<8; i++) {
		dma_buf[i] = ow_bits[byte & 0x01];
		byte >>= 1;
	}

	wire->USARTx->SR = (uint16_t) ~(USART_FLAG_RXNE | USART_FLAG_TC); // Clear flags
	wire->USARTx->CR3 |= USART_DMAReq_Tx | USART_DMAReq_Rx; // Enable RX/TX DMA
	wire->dma_rx->CCR |= DMA_CCR1_EN;
	wire->dma_tx->CCR |= DMA_CCR1_EN;

	while (!(DMA1->ISR & wire->dma_rx_flag)) {
#ifdef OW_FREERTOS
		taskYIELD();
#endif
	}

	wire->dma_tx->CCR &= ~DMA_CCR1_EN;
	wire->dma_rx->CCR &= ~DMA_CCR1_EN;
	wire->USARTx->CR3 &= (uint16_t) ~(USART_DMAReq_Tx | USART_DMAReq_Rx);
	//USART_ClearFlag(OW_USART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
	return OW_OK;
}//*/
#endif /* OW_MODE_USART_DMA */

void owu_skip(owu_struct_t *wire)
{
	owu_write_byte(wire, 0xCC);
}

void owu_select_device(owu_struct_t *wire, const uint8_t *addr)
{
	owu_write_byte(wire, 0x55);
	uint8_t i;
	for (i=0; i<8; i++) {
		owu_write_byte(wire, *addr++);
	}
}

#ifdef OW_LIB_SEARCH
void owu_reset_search(owu_struct_t *wire)
{
	wire->last_discrepancy = 0;
	wire->last_device_flag = 0;
	wire->last_family_discrepancy = 0;
	memset(&wire->rom_no[0], 0, 8);
}

uint8_t owu_search(owu_struct_t *wire, uint8_t *addr)
{
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number, search_result;
	uint8_t id_bit, cmp_id_bit;
	uint8_t rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = 0;

	// if the last call was not the last one
		if (!wire->last_device_flag)
		{
			// 1-Wire reset
			if (owu_reset(wire) == OW_NOTHING)
			{
				// reset the search
				wire->last_discrepancy = 0;
				wire->last_device_flag = 0;
				wire->last_family_discrepancy = 0;
				//print_usart(USART1, "Search::no devices\r\n");
				return 0;
			}

			// issue the search command
			owu_write_byte(wire, 0xF0);

			// loop to do the search
			do
			{
				// read a bit and its complement
				id_bit = owu_read_bit(wire);
				cmp_id_bit = owu_read_bit(wire);

				// check for no devices on 1-wire
				if ((id_bit == 1) && (cmp_id_bit == 1)) {
					break;
				}
				else
				{
					// all devices coupled have 0 or 1
					if (id_bit != cmp_id_bit) {
						search_direction = id_bit;  // bit write value for search
					}
					else
					{
						// if this discrepancy if before the Last Discrepancy
						// on a previous next then pick the same as last time
						if (id_bit_number < wire->last_discrepancy) {
							search_direction = ((wire->rom_no[rom_byte_number] & rom_byte_mask) > 0);
						}
						else {
							// if equal to last pick 1, if not then pick 0
							search_direction = (id_bit_number == wire->last_discrepancy);
						}
						// if 0 was picked then record its position in LastZero
						if (search_direction == 0)
						{
							last_zero = id_bit_number;

							// check for Last discrepancy in family
							if (last_zero < 9) {
								wire->last_family_discrepancy = last_zero;
							}
						}
					}

					// set or clear the bit in the ROM byte rom_byte_number
					// with mask rom_byte_mask
					if (search_direction == 1) {
					  wire->rom_no[rom_byte_number] |= rom_byte_mask;
					}
					else {
					  wire->rom_no[rom_byte_number] &= ~rom_byte_mask;
					}

					// serial number search direction write bit
					owu_write_bit(wire, search_direction);

					// increment the byte counter id_bit_number
					// and shift the mask rom_byte_mask
					id_bit_number++;
					rom_byte_mask <<= 1;

					// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
					if (rom_byte_mask == 0)
					{
						 rom_byte_number++;
						 rom_byte_mask = 1;
					}
				}
			}
			while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

			// if the search was successful then
			if (!(id_bit_number < 65))
			{
				// search successful so set wire->last_discrepancy,wire->last_device_flag,search_result
				wire->last_discrepancy = last_zero;

				// check for last device
				if (wire->last_discrepancy == 0)
					wire->last_device_flag = 1;

				search_result = 1;
			}
		}

		// if no device found then reset counters so next 'search' will be like a first
		if (!search_result || !wire->rom_no[0])
		{
			wire->last_discrepancy = 0;
			wire->last_device_flag = 0;
			wire->last_family_discrepancy = 0;
			search_result = 0;
			//print_usart(USART1, "Search::result reset\r\n");
		} else {
			/*for (int i = 0; i < 8; i++) {
				addr[i] = wire->rom_no[i];
			}*/
			memcpy(addr, wire->rom_no, 8);
		}

		return search_result;
}
#endif /* OW_LIB_SEARCH */
