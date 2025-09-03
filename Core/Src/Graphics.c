/*
 * Graphics.c
 *
 *  Created on: Aug 2, 2025
 *      Author: Jerem
 */
#include "Graphics.h"
#include <stdint.h>
#include "main.h"
#include "spi.h"
#include "SystemTimer.h"
/*
 * 	unsigned int character_width = getCharacterPhysicalWidth(c);
	if (*column + character_width >= 132) {
		*page = *page + 1;
		*column = 0;

		if (*page > 3) *page = 0;
	}

	uint8_t* character_pixels = getCharacterPixels(c);

	memcpy(LCD_BUFFER + *page * 132 + *column, character_pixels, character_width);

	setAddress(*page, *column);
	sendData(character_pixels, character_width);

	*column += character_width + CHARACTER_SPACING;
 */

void show_notification(char* notification) {
	clearSegment(0, 0, 132);
	uint8_t page = 0;
	uint8_t col = 0;

	uint8_t character_copy[6];
	while (*notification != '\0') {
		char c = *notification;
		uint8_t* character_pixels = getCharacterPixels(c);

		unsigned int width = getCharacterPhysicalWidth(c);

		memcpy(character_copy, character_pixels, width);

		for (unsigned int i = width; i < 6; i++) character_copy[i] = 0; // fill extra space with 0s

		for (unsigned int i = 0; i < 6; i++) character_copy[i] = (character_copy[i] >> 1) | 0x80;

		setAddress(page, col);
		sendData(character_copy, width + 1);

		col += width + 1;
		notification++;
	}

	uint8_t data[5];
	data[0] = 0xC0;
	for (unsigned int i = 1; i < 5; i++)
		data[i] = data[i - 1] >> 2;

	setAddress(0, col);
	sendData(data, 5);

	//register_timer(5, hide_notification, 0);
}

void hide_notification() {
	setAddress(0, 0);
	sendData(LCD_BUFFER, 132);
}
