/*
 * rotary.c
 *
 *	Decoding the direction of the rotary encoder.
 *
 *  Created on: Jan 18, 2016
 *      Author: Darau, blė
 */

#include "rotary.h"

// An array of rotary encoder pulse transisions
// using hysteresis of previously read pulses
const rotary_state_type states[] = {
#if (ROTARY_PULL_UP == 1)
	IDLE, // 0
	CW, // 1
	CCW, // 2
	IDLE, // 3
	IDLE, // 4
	IDLE, // 5
	CCW, // 6
	CW, // 7
	IDLE, // 8
	CW, // 9
	CCW, // 10
	CCW, // 11
	IDLE, // 12
	IDLE, // 13
	IDLE, // 14
	IDLE, // 15
#else
	IDLE, // 0
	IDLE, // 1
	IDLE, // 2
	IDLE, // 3
	CCW, // 4
	CCW, // 5
	CW, // 6
	IDLE, // 7
	CW, // 8
	CCW, // 9
	IDLE, // 10
	IDLE, // 11
	IDLE, // 12
	CCW, // 13
	CW, // 14
	IDLE, // 15
#endif
};

const int rotary_direction[] = {
	CCW, // -2
	IDLE, // -1
	IDLE, // 0
	IDLE, // 1
	CW, // 2
};

rotary_state_type read_ab(encoder_t* encoder, uint8_t a, uint8_t b)
{
	// Read rotary encoder pins, shift previous states and sum up. Result will
	// provide a valid rotation from the transition array.
	encoder->rotary_state = (encoder->rotary_state << 2 | (a << 1) | b) & 0x0F;

	encoder->hsys[0] = encoder->hsys[1];
	encoder->hsys[1] = states[encoder->rotary_state];
	// Damping of rotary fluctuations, when wrong directions is read accidentally
	return rotary_direction[encoder->hsys[0]+encoder->hsys[1]+2];
}

rotary_button_type read_button(encoder_t* encoder, uint8_t p) {
	encoder->button_state = ((encoder->button_state << 1) | p) & 0x03;
	if (encoder->button_state == 0x01) {
#if (ROTARY_PULL_UP == 1)
		return ON;
	} else {
		return OFF;
#else
		return OFF;
	} else {
		return ON;
#endif
	}
}
