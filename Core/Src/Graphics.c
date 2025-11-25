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

#include <string.h>
#include "keyboard_icons.h"
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


void show_menu(char* icons, unsigned int total_icons, unsigned int page) {
	if (page > total_icons / 5) page = 0; // make sure its not out of bounds

	int icon_index = page * 5;

	uint8_t* menu_buffer = malloc(132);

	uint8_t* draw_ptr = menu_buffer + 1;

	if (menu_buffer == 0) return;

	memset(menu_buffer, 0, 132);

	for (int icon_index = page * 5; icon_index < (page + 1) * 5 && icon_index < total_icons; icon_index++) {
		memcpy(draw_ptr, icons + icon_index * 25, 25);

		draw_ptr += 26;
	}

	setAddress(3, 0);
	sendData(menu_buffer, 132);

	free(menu_buffer);
}

int text_entry(char* result, unsigned int max_result, char* message) {
	uint8_t succeeded = 0;

	uint8_t* frame_copy = malloc(132 * 4);

	if (frame_copy == 0) return 1;

	memcpy(frame_copy, LCD_BUFFER, 132 * 4);

	uint8_t zero = 0;

	uint8_t page = 0;
	uint8_t col = 0;

	for (int c = 0; c < 132; c++) {
		for (int p = 0; p < 4; p++) {
			setAddress(p, c);
			sendData(&zero, 1);
		}
	}

	memset(result, 0, max_result);


	page = 0;
	col = 0;
	RP_PRINT_TEXT(message, &page, &col);


	int next_character_index = 0;

	int current_menu = 0;

	char* menus[] = {
			alpha, abcde, fghi, jklm, nopq, rstuv, wxyz
	};

	int menu_lengths[] = {
			5, 4, 4, 4, 4, 4
	};

	char* start_chars = "AFJNRW";

	while (1) {
		page = 3;
		col = 0;
		RP_DRAW_ICON(menus[current_menu], 132, 1, &page, &col);
		page = 1;
		col = 0;

		for (int c = 0; c < 132; c++) {
			setAddress(1, c);
			sendData(&zero, 1);
		}

		RP_PRINT_TEXT(" > ", &page, &col);
		RP_PRINT_TEXT(result, &page, &col);

		uint8_t key = system_call(2, 0);

		// probably a better way to do this :/
		if (key > 6 && key != 13 && key != 17 && key != 33 && key != 34 && key != 31 && key != 30 && key != 29 && key != 26 && key != 25 && key != 24 && key != 21 && key != 20 && key != 19) continue;

		if (key <= 6) { // menu select
			if (current_menu == 0) {
				current_menu = key;
				continue;
			}

			char c = start_chars[current_menu - 1] + (key - 1);

			if (key - 1 >= menu_lengths[current_menu - 1]) c = ' ';

			if (next_character_index < max_result - 1)
				result[next_character_index++] = c;

			current_menu = 0;
			continue;
		} else if (key == 13) { // enter
			if (result[0] == '\0') {
				succeeded = 0;// it is blank so cancel operation
				goto end;
			}
			succeeded = 1;
			goto end;  // result in the result buffer
		} else if (key == 17) { // backspace
			if (next_character_index > 0) result[--next_character_index] = '\0';
		} else if (key == 33) {
			if (current_menu != 0) current_menu = 0;
			else {
				result[0] = '\0';
				succeeded = 0;
				goto end;
			}
		} else { // numeric
			char next_char = '0';

			switch (key) {
			case 34:
				next_char = '0';
				break;
			case 31:
				next_char = '3';
				break;
			case 30:
				next_char = '2';
				break;
			case 29:
				next_char = '1';
				break;
			case 26:
				next_char = '6';
				break;
			case 25:
				next_char = '5';
				break;
			case 24:
				next_char = '4';
				break;
			case 21:
				next_char = '9';
				break;
			case 20:
				next_char = '8';
				break;
			case 19:
				next_char = '7';
				break;
			default:
				continue;
			}

			if (next_character_index < max_result - 1)
				result[next_character_index++] = next_char;
		}
	}

	end:
	memcpy(LCD_BUFFER, frame_copy, 132 * 4);
	UpdateLCD();
	free(frame_copy);

	return succeeded;
}


void hide_menu() {
	setAddress(3, 0);
	sendData(LCD_BUFFER, 132);
}
