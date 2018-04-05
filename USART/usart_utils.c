/*
 * Print out strings to the selected U(S)ART of the STM32F10x processors.
 *
 * usart_utils.c
 *
 *  TODO: only USART1, USART2, 3 are implemented. UARTs 4 and 5 for STM32F107 are not yet done.
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
#include "usart_utils.h"
#include "makrosai.h"

#define KR_KEY_Reload    ((uint16_t)0xAAAA)
#define KR_KEY_Enable    ((uint16_t)0xCCCC)

// Adjusted to 72 MHz clock
const uint32_t bauds[] =  { 1200,   4800,   9600,   19200,  38400,  57600,  115200 };
const uint16_t brr_u1[] = { 0xEA60, 0x3A98, 0x1D4C, 0x0EA6, 0x0753, 0x04E2, 0x0271 };
const uint16_t brr_ux[] = { 0x7530, 0x1D4C, 0x0EA6, 0x0753, 0x03A9, 0x0271, 0x0138 };


void enable_usart(USART_TypeDef *USARTx, uint32_t baud_rate) {
	RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN;
	if      (USARTx == USART1) {
		RCC->APB2ENR |= RCC_APB2RSTR_USART1RST;
		RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;
		GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 | GPIO_CRH_CNF10 | GPIO_CRH_MODE10); // Clear configuration
		GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 | GPIO_CRH_CNF10_0; // Alternative Push-pull, 50 MHz; in floating
		//*/
	}
	else if (USARTx == USART2) {
		RCC->APB1ENR |= RCC_APB1RSTR_USART2RST;
		RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;
		GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2 | GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
		GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2 | GPIO_CRL_CNF3_0;
	} else if (USARTx == USART3) {
		RCC->APB1ENR |= RCC_APB1RSTR_USART3RST;
		RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;
		GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
		GPIOB->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10 | GPIO_CRH_CNF11_0;
	}
	else if (USARTx == UART4) {
		// TODO implement
	}
	else if (USARTx == UART5) {
		// TODO implement
	}

	// Clear config
	USARTx->CR1 = 0;
	USARTx->CR2 = 0;
	USARTx->CR3 = 0;
	USARTx->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;


	uint8_t i;
	for (i=0; i<7; i++) {
		if (bauds[i] == baud_rate) {
			if (USARTx == USART1) {
				USARTx->BRR = brr_u1[i];
			} else {
				USARTx->BRR = brr_ux[i];
			}
			break;
		}
		if (i == 6) {
			// default to 9600
			if (USARTx == USART1) {
				USARTx->BRR = brr_u1[2];
			} else {
				USARTx->BRR = brr_ux[2];
			}
		}
	}
}

// TODO: implement
void disable_usart(USART_TypeDef *USARTx){
	USARTx->CR1 &= ~USART_CR1_UE;

	if      (USARTx == USART1) { RCC->APB2ENR &= ~RCC_APB2RSTR_USART1RST; }
	else if (USARTx == USART2) { RCC->APB1ENR &= ~RCC_APB1RSTR_USART2RST; }
	else if (USARTx == USART3) { RCC->APB1ENR &= ~RCC_APB1RSTR_USART3RST;; }
	else if (USARTx == UART4)  { RCC->APB1ENR &= ~RCC_APB1Periph_UART4; }
	else if (USARTx == UART5)  { RCC->APB1ENR &= ~RCC_APB1Periph_UART5; }
}

int print_usart(USART_TypeDef *USARTx, const char *str) {
	int i = 0;
	while (*str) {
		IWDG->KR = KR_KEY_Reload;
		// Write a character to the USART
		USARTx->DR = (*str & USART_DR_DR);

		while(!(USARTx->SR & USART_SR_TXE)) {
			// Reset IWD
			IWDG->KR = KR_KEY_Reload;
		}
		i++;
		str++;
	}
	return i;
}
