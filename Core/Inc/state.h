/*
 * state.h
 *
 *  Created on: Aug 8, 2025
 *      Author: Jerem
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

#include <stdint.h>

#define POWER_USB     0b00000000
#define POWER_BATTERY 0b00000001

extern uint8_t power_state;

#define CLOCK_HIGH 0b00000001
#define CLOCK_LOW  0b00000010

extern uint8_t clock_state;

#endif /* INC_STATE_H_ */
