/*
 * The library to read DHT22 temperature sensor data. As the sensor use variable length pulses
 * to output data, it is read using simple bitbanging of GPIO pin and microseconds delays.
 *
 * dht.h
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

#ifndef INCLUDE_DHT_H_
#define INCLUDE_DHT_H_

#include <sys/cdefs.h>
#include "stm32f10x.h"


// Temperature conversion metrics
#define CELSIUS 0
#define KELVIN 1
#define FARENHEIT 2

#define TIMEOUT 100000

#define DHT_OK 0
#define DHT_ERR_CHECKSUM 1
#define DHT_TIMEOUT 2
#define DHT_INVALID -999.0;

typedef struct {
	GPIO_TypeDef *gpio;
	volatile uint32_t *cr;
	uint32_t pin;
	uint32_t clear_mask;
	uint32_t in_mask;
	uint32_t out_mask;
	double temperature;
	double humidity;
} dht_t;

typedef struct {

} dht_data_t;

__BEGIN_DECLS

/**
 * External function providing microseconds delay
 */
extern void delay_us(uint32_t);

/**
 * Initialize DHT22 reading structure
 *
 * @param: GPIO to be used (e.g. GPIOA, GPIOB etc.)
 * @param: pin number to be used (0-15)
 * @param: DHT22 describing structure to be initialized
 */
void init_dht(GPIO_TypeDef *gpio, uint16_t pin, dht_t *dht);


/**
 * Read the DHT22 sensor data and set temperature and humidity
 * in the provided structure
 *
 * @param: initialized DHT22 describing structure
 * @param: temperature conversion metrics, e.g. CELSIUS, KELVIN or FARENTHEIT
 */
uint8_t read_dht22(dht_t *dht, uint8_t metrics);

__END_DECLS

#endif /* INCLUDE_DHT_H_ */
