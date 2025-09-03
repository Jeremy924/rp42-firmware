/*
 * SystemTimer.h
 *
 *  Created on: Aug 2, 2025
 *      Author: Jerem
 */

#ifndef INC_SYSTEMTIMER_H_
#define INC_SYSTEMTIMER_H_

#include <stdint.h>

struct TimerRegisterArgs {
	uint32_t millis;
	void (*function)();
	uint8_t flags;
};

uint8_t register_timer(uint32_t millis, void (*function)(uint8_t), uint8_t flags);
void unregister_timer(uint8_t handle);

void handle_tick();

#endif /* INC_SYSTEMTIMER_H_ */
