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

	uint8_t command[] = {0b10110010, 0b00010000, 0b00000000};
	sendCommand(&command, sizeof(command));
	sendData(temp_buf, 132);
}

#define MIN(a, b) a < b ? a : b
#define BAR_MAX_PHYSICAL_LENGTH 25
#define DISPLAY_WIDTH 132
/**
 * @brief Displays a continuous progress bar that grows and then slides.
 * @param last_progress_state Pointer to a uint8_t variable holding the current state (0-255).
 * This state variable will be incremented by the function.
 */
void continuous_progress_bar(uint8_t* last_progress_state) {
    // Calculate the current logical "head" position of the progress.
    // head_position will range from 0 to (DISPLAY_WIDTH - 1) as *last_progress_state goes 0-255.
    unsigned int head_position = (*last_progress_state * (unsigned long)DISPLAY_WIDTH) / 256;

    uint8_t temp_buf[DISPLAY_WIDTH];
    memset(temp_buf, 0, DISPLAY_WIDTH); // Clear the display buffer

    // Determine the actual length of the bar segment to draw.
    // It grows until it reaches BAR_MAX_PHYSICAL_LENGTH, then stays that length.
    unsigned int current_bar_segment_length = MIN(head_position + 1, BAR_MAX_PHYSICAL_LENGTH);
    // Note: Using head_position + 1 because if head_position is 0, we want a bar of length 1.
    // If head_position is 0, MIN(1, 25) = 1.
    // If head_position is 24, MIN(25, 25) = 25.
    // If head_position is 25, MIN(26, 25) = 25.
    // If head_position is 131, MIN(132, 25) = 25.

    // Determine the starting offset for drawing the bar segment.
    // During the growth phase (head_position < BAR_MAX_PHYSICAL_LENGTH), the bar starts at 0.
    // During the sliding phase, the bar starts at (head_position - BAR_MAX_PHYSICAL_LENGTH + 1).
    unsigned int start_draw_offset;
    if ((head_position + 1) <= BAR_MAX_PHYSICAL_LENGTH) { // Growth phase
        start_draw_offset = 0;
        // current_bar_segment_length is already head_position + 1 here
    } else { // Sliding phase
        start_draw_offset = (head_position + 1) - BAR_MAX_PHYSICAL_LENGTH;
        // current_bar_segment_length is BAR_MAX_PHYSICAL_LENGTH
    }

    // If head_position is 0, current_bar_segment_length = 1, start_draw_offset = 0. Draws 1 pixel at [0].
    // If head_position is 24 (CBL=25), current_bar_segment_length = 25, start_draw_offset = 0. Draws 25 pixels at [0..24].
    // If head_position is 25 (CBL=25), current_bar_segment_length = 25, start_draw_offset = 1. Draws 25 pixels at [1..25].
    // If head_position is 131 (CBL=25), current_bar_segment_length = 25, start_draw_offset = 107. Draws 25 pixels at [107..131].


    // Draw the bar segment if its length is greater than 0
    if (current_bar_segment_length > 0) {
        // Ensure the offset is within bounds (it should be by calculation)
        if (start_draw_offset < DISPLAY_WIDTH) {
            // Ensure we don't write past the end of temp_buf
            unsigned int bytes_to_fill = MIN(current_bar_segment_length, DISPLAY_WIDTH - start_draw_offset);
            memset(temp_buf + start_draw_offset, 255, bytes_to_fill);
        }
    }

    setAddress(2, 0); // Set display address (assuming for the entire line)
    sendData(temp_buf, DISPLAY_WIDTH); // Send the buffer to the display

    *last_progress_state += 1; // Increment state for next frame (uint8_t will wrap naturally)
}
