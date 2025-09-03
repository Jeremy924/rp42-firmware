/*
 * SystemTimer.c
 *
 *  Created on: Aug 2, 2025
 *      Author: Jerem
 */

#include "SystemTimer.h"
#include "tim.h"
/*
 * 	__HAL_TIM_SET_COUNTER(&htim16, 0);
	uint32_t new_period = (systemConfigData.debounce_ms * 10) - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim16, new_period);

	HAL_TIM_Base_Start_IT(&htim16);
 */

#define HANDLES_LIMIT 10

#define REGISTER_SUCCESS 0
#define TOO_MANY_TIMERS  1

struct SystemTimer {
	uint8_t flags;
	void (*function)(uint8_t);
	uint32_t millis_to_wait;
};


#define TIMER_RUNNING 0b00000001
uint8_t timer_status;
uint8_t controlling_handle;
uint32_t last_time_ran;

struct SystemTimer timers[HANDLES_LIMIT];


void init_timer_system() {
	for (int i = 0; i < HANDLES_LIMIT; i++) {
			timers[i].function = 0;
	}

	timer_status = 0;
	controlling_handle = 0xff;

	MX_TIM15_Init();
}

// seconds <= 5
void _start_timer(uint32_t millis) {
	if (millis == 0) millis = 1;

	__HAL_TIM_SET_COUNTER(&htim15, 0);
	uint32_t new_period = millis - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim15, new_period);

	timer_status |= TIMER_RUNNING;
	last_time_ran = millis;
    __HAL_TIM_CLEAR_IT(&htim15, TIM_IT_UPDATE);

    HAL_TIM_Base_Stop_IT(&htim15);
	if (HAL_TIM_Base_Start_IT(&htim15) != HAL_OK) Error_Handler();
}

void _stop_timer() {
	HAL_TIM_Base_Stop_IT(&htim15);
	timer_status &= ~TIMER_RUNNING;
}

uint32_t process_time_elapsed(uint32_t millis_elapsed) {
	uint32_t next_millis = 0xffffffff;

	for (unsigned int i = 0; i < HANDLES_LIMIT; i++) {
		struct SystemTimer* timer = &timers[i];
		if (timer->function == 0) continue;

		timer->millis_to_wait -= millis_elapsed;
		if (timer->millis_to_wait <= 0) {
			timer->function(i);
			timer->function = 0;
		} else if (timer->millis_to_wait < next_millis) {
			next_millis = timer->millis_to_wait;
		}
	}

	return next_millis;
}

void handle_tick() {
	uint32_t next_millis = process_time_elapsed(last_time_ran);

	timer_status &= ~TIMER_RUNNING;
	if (next_millis != 0xffffffff) _start_timer(next_millis);
}

void unregister_timer(uint8_t handle) {
	if (handle >= HANDLES_LIMIT) return;

	timers[handle].function = 0;

	if (handle == controlling_handle) {
		_stop_timer();
		last_time_ran = htim15.Instance->CNT;

		if (last_time_ran != 0) handle_tick();
	}
}

uint8_t register_timer(uint32_t millis, void (*function)(uint8_t), uint8_t flags) {
	unsigned int handle = 0xffffffff;

	for (unsigned int i = 0; i < HANDLES_LIMIT; i++) {
		if (timers[i].function == 0) {
			handle = i;
			break;
		}
	}

	uint32_t remaining_millis = htim15.Instance->ARR - htim15.Instance->CNT;

	if (remaining_millis > millis) {
		_stop_timer();
		last_time_ran = htim15.Instance->CNT;
		process_time_elapsed(last_time_ran);
		timer_status &= ~TIMER_RUNNING;

		controlling_handle = handle;
	}

	if (handle == 0xffffffff) { // no more timers available
		return handle;
	}

	timers[handle].flags = flags;
	timers[handle].millis_to_wait = millis;
	__disable_irq();
	timers[handle].function = function;
	__enable_irq();

	if ((timer_status & TIMER_RUNNING) == 0) {
		controlling_handle = handle;
		_start_timer(millis);
	}


	return handle;
}
