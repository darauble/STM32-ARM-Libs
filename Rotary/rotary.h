/*
 * rotary.h
 *  Small and quite efficient rotary encoder reading library,
 *  independent from interrupts. Reading must be simply performed
 *  inside interrupt routine(s).
 *
 *  Library is designed to be used with C pin of the rotary encoder
 *  connected to either +3.3V or the GND, see define below.
 *
 *  Created on: Jan 18, 2016
 *      Author: Darau, blė
 */

#ifndef ROTARY_H_
#define ROTARY_H_

#include <stdint.h>

/* Rotary encoder can short GPIO to either ground or VCC.
 * When GPIO is pulled down by default and encoder pulls it up,
 * then define `ROTARY_PULL_UP 1`. It is more common to have
 * GPIO pulled up and buttons/encoders pull them down.
 */
#ifndef ROTARY_PULL_UP
#define ROTARY_PULL_UP 0
#endif

// State enumeration of the rotary encoder.
typedef enum {
	CCW = -1,
	IDLE = 0,
	CW = 1
} rotary_state_type;

typedef enum {
	OFF = 0,
	ON
} rotary_button_type;

/* Description type of one connected rotary encoder.
 * Provide the port and pin for A and B pins of the rotary encoder.
 */
typedef struct {
	int8_t hsys[2];
	uint8_t rotary_state;
	uint8_t button_state;
	uint32_t on_time; // Use this variable to detect length of the push on button
} encoder_t;

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Call this routine with described rotary encoder.
 *
 * @param: a rotary encoder structure
 * @param: a value of "A" pin (1 or 0)
 * @param: a value of "B" pin (1 or 0)
 * @return: one of the rotary encoder states
 */
rotary_state_type read_ab(encoder_t* encoder, uint8_t a, uint8_t b);

/**
 * Call this routine with described rotary encoder.
 *
 * @param: a rotary encoder with described GPIO pins
 * @return: push button state
 */
rotary_button_type read_button(encoder_t* encoder, uint8_t p);
#if defined(__cplusplus)
} // extern "C"
#endif

#endif /* ROTARY_H_ */
