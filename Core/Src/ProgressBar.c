/*
 * ProgressBar.c
 *
 *  Created on: May 14, 2025
 *      Author: Jerem
 */
#include "ProgressBar.h"

void init_progress_bar() {
	memset(LCD_BUFFER + 132, 0, 132*3);

	UpdateLCD();
}

void set_progress(uint8_t progress) {
	unsigned int length = progress * 132 / 256;
	if (length > 131) length = 131;

	uint8_t temp_buf[132];
	memset(temp_buf + length, 0, 132 - length);
	memset(temp_buf, 255, length);

	uint8_t command[] = {0b10110001, 0b00010000, 0b00000000};
	sendCommand(&command, sizeof(command));
	sendData(temp_buf, 132);
}
#define CONTINUOUS_BAR_LENGTH 25
void continuous_progres_bar(uint8_t* last) {
	unsigned int length = *last * (132-CONTINUOUS_BAR_LENGTH) / 256;

	memset(LCD_BUFFER, 0, 132);
	memset(LCD_BUFFER + length - CONTINUOUS_BAR_LENGTH, 255, CONTINUOUS_BAR_LENGTH);

	*last += 1;

	UpdateLCD();
}
