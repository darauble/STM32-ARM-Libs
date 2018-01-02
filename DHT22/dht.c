/*
 * The library to read DHT22 temperature sensor data. As the sensor use variable length pulses
 * to output data, it is read using simple bitbanging of GPIO pin and microseconds delays.
 *
 * dht.c
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

#include <string.h>
#include "dht.h"

// For debugging only!
/*#include "usart_utils.h"
#include <stdio.h>
extern char buf[100];//*/

static uint8_t dht_bytes[5];

static uint8_t read(dht_t *dht);

void init_dht(GPIO_TypeDef *gpio, uint16_t pin, dht_t *dht)
{
	dht->gpio = gpio;
	dht->pin = 0x01 << pin;

	if (pin < 8) {
		dht->cr = &gpio->CRL;
		dht->clear_mask = (GPIO_CRL_CNF0 | GPIO_CRL_MODE0) << (4 * pin);
		dht->in_mask = (GPIO_CRL_CNF0_0) << (4 * pin);
		dht->out_mask = (GPIO_CRL_CNF0_0 | GPIO_CRL_MODE0) << (4 * pin);
	} else {
		dht->cr = &gpio->CRH;
		dht->clear_mask = (GPIO_CRH_CNF8 | GPIO_CRH_MODE8) << (4 * (pin-8));
		dht->in_mask = (GPIO_CRH_CNF8_0) << (4 * (pin-8));
		dht->out_mask = (GPIO_CRH_CNF8_0 | GPIO_CRH_MODE8) << (4 * (pin-8));
	}

	if (gpio == GPIOA) {
		RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;
	} else if (gpio == GPIOB) {
		RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;
	}
}

uint8_t read_dht22(dht_t *dht, uint8_t metrics)
{
	uint8_t status = read(dht);
	if (status != DHT_OK) {
		dht->temperature = DHT_INVALID;
		dht->humidity = DHT_INVALID;
		return status;
	}
	// For debugging only!
	/*snprintf(buf, 99, "DHT: %02X %02X %02X %02X %02X\r\n", dht_bytes[0], dht_bytes[1], dht_bytes[2], dht_bytes[3], dht_bytes[4]);
	print_usart(USART1, buf);//*/

	dht->humidity = ((dht_bytes[0] << 8) | dht_bytes[1]) * 0.1;

	if (dht_bytes[2] & 0x80) {
		// Negative temperature
		dht->temperature = -0.1 * (((dht_bytes[2] & 0x7F) << 8) | dht_bytes[3]);
	} else {
		dht->temperature = 0.1 * ((dht_bytes[2] << 8) | dht_bytes[3]);
	}

	// Checksum
	uint8_t sum = dht_bytes[0] + dht_bytes[1] + dht_bytes[2] + dht_bytes[3];
	if (dht_bytes[4] != sum) {
		dht_bytes[4] &= 0xFE;
		if (dht_bytes[4] != sum) {
			return DHT_ERR_CHECKSUM;
		}
	}

	if (metrics == KELVIN) {
		dht->temperature += 273.15;
	} else if (metrics == FARENHEIT) {
		dht->temperature = dht->temperature * 1.8 + 32;
	} // Default is Celsius

	return DHT_OK;
}

static uint8_t read(dht_t *dht)
{
	memcpy(dht_bytes, 0, 5);
	*dht->cr &= ~dht->clear_mask;
	*dht->cr |= dht->out_mask;

	// Send the master pulse to initiate DHT sensor output
	dht->gpio->BRR |= dht->pin;
	uint8_t i;
	for (i=0; i<20; i++) {
		delay_us(1000);
	}
	dht->gpio->BSRR |= dht->pin;
	delay_us(40);

	*dht->cr &= ~dht->clear_mask;
	*dht->cr |= dht->in_mask;

	uint32_t lo_cnt = TIMEOUT;
	uint32_t hi_cnt = TIMEOUT;

	// Check for HIGH line
	while (dht->gpio->IDR & dht->pin) {
		if (hi_cnt-- < 1) {
			//return DHT_TIMEOUT;
			return 11;
		}
	}
	// Read the ACK pulse
	while (!(dht->gpio->IDR & dht->pin)) {
		if (lo_cnt-- < 1) {
			//return DHT_TIMEOUT;
			return 10;
		}
	}
	while (dht->gpio->IDR & dht->pin) {
		if (hi_cnt-- < 1) {
			//return DHT_TIMEOUT;
			return 12;
		}
	}

	// Now read 40 bits (5 bytes) output from DHT sensor. If HIGH is longer than LOW,
	// then 1 is read, otherwise 0.
	int8_t j;
	for (i=0; i<5; i++) {
		for (j=7; j>=0; j--) {
			lo_cnt = TIMEOUT;
			hi_cnt = TIMEOUT;

			while (!(dht->gpio->IDR & dht->pin)) {
				if (lo_cnt-- < 1) {
					return DHT_TIMEOUT;
				}
			}
			while (dht->gpio->IDR & dht->pin) {
				if (hi_cnt-- < 1) {
					return DHT_TIMEOUT;
				}
			}

			if (hi_cnt < lo_cnt) {
				// High pulse was longer than low, it is 1
				dht_bytes[i] |= (0x01 << j);
			}
		}
	}
	return DHT_OK;
}
