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

#define CR1_CLEAR_Mask				((uint16_t)0xE9F3)  /*!< USART CR1 Mask */
#define CR2_STOP_CLEAR_Mask		 ((uint16_t)0xCFFF)  /*!< USART CR2 STOP Bits Mask */
#define CR3_CLEAR_Mask				((uint16_t)0xFCFF)  /*!< USART CR3 Mask */


// Adjusted to 72 MHz clock
const uint32_t bauds[] =  { 1200,   4800,   9600,   19200,  38400,  57600,  115200 };
const uint16_t brr_u1[] = { 0xEA60, 0x3A98, 0x1D4C, 0x0EA6, 0x0753, 0x04E2, 0x0271 };
const uint16_t brr_ux[] = { 0x7530, 0x1D4C, 0x0EA6, 0x0753, 0x03A9, 0x0271, 0x0138 };

static void enable_usart_full(USART_TypeDef *USARTx, uint32_t baud_rate, uint32_t USART_Periph, GPIO_TypeDef *GPIOx, uint16_t tx_pin, uint16_t rx_pin) {
	USART_InitTypeDef usartConfig;
	GPIO_InitTypeDef gpioConfig;

	if (USARTx == USART1) {
		//RCC_APB2PeriphClockCmd(USART_Periph, ENABLE);
		RCC->APB2ENR |= RCC_APB2RSTR_USART1RST;
	} else if (USARTx == USART2) {
		RCC->APB1ENR |= RCC_APB1RSTR_USART2RST;
	}
#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
	else if (USARTx == USART3) {
		RCC->APB1ENR |= RCC_APB1RSTR_USART3RST;
	}
#endif
#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL)
	else if (USARTx == UART4) {
		RCC->APB1ENR |= RCC_APB1RSTR_UART4RST;
	} else if (USARTx == UART5) {
		RCC->APB1ENR |= RCC_APB1RSTR_UART5RST;
	}
#endif
	if (GPIOx == GPIOA) {
		GPIOA_ENABLE;
	}

	gpioConfig.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioConfig.GPIO_Pin = tx_pin;
	gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &gpioConfig);

	//PA10 = USART1.RX => Input
	gpioConfig.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioConfig.GPIO_Pin = rx_pin;
	GPIO_Init(GPIOx, &gpioConfig);


	usartConfig.USART_BaudRate = baud_rate;
	usartConfig.USART_WordLength = USART_WordLength_8b;
	usartConfig.USART_StopBits = USART_StopBits_1;
	usartConfig.USART_Parity = USART_Parity_No;
	usartConfig.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartConfig.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USARTx, &usartConfig);
	// Enable usart
	USARTx->CR1 |= USART_CR1_UE;
	//USART_Cmd(USARTx, ENABLE);
}

void enable_usart(USART_TypeDef *USARTx, uint32_t baud_rate) {
	RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN;
	if      (USARTx == USART1) { //enable_usart_full(USARTx, baud_rate, RCC_APB2Periph_USART1, GPIOA, GPIO_Pin_9, GPIO_Pin_10);
		RCC->APB2ENR |= RCC_APB2RSTR_USART1RST;
		RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;
		GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 | GPIO_CRH_CNF10 | GPIO_CRH_MODE10); // Clear configuration
		GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9 | GPIO_CRH_CNF10_0; // Alternative Push-pull, 50 MHz; in floating
		//*/
	}
	else if (USARTx == USART2) {
		//RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		//enable_usart_full(USART2, baud_rate, RCC_APB1Periph_USART2, GPIOA, GPIO_Pin_2, GPIO_Pin_3);
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

	USARTx->CR1 |= USART_CR1_UE;
	USARTx->CR2 &= CR2_STOP_CLEAR_Mask; // Clear CR2 stop bits, leave as is for Stop Bits 1
	USARTx->CR1 &= CR1_CLEAR_Mask; // Clear CR1 config
	USARTx->CR1 |= USART_Mode_Rx | USART_Mode_Tx; // 8b word, no parity, RX/TX
	USARTx->CR3 &= CR3_CLEAR_Mask; // Clear CR3 config, leave as is, no hw control

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
	USART_Cmd(USARTx, DISABLE);

	if      (USARTx == USART1) { RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE); }
	else if (USARTx == USART2) { RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE); }
	else if (USARTx == USART3) { RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE); }
	else if (USARTx == UART4)  { RCC_APB2PeriphClockCmd(RCC_APB1Periph_UART4, DISABLE); }
	else if (USARTx == UART5)  { RCC_APB2PeriphClockCmd(RCC_APB1Periph_UART5, DISABLE); }
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
		IWDG->KR = KR_KEY_Reload;
		i++;
		str++;
	}
	return i;
}
