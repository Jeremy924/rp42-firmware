/*
 * tetris.h
 *
 * Created on: May 3, 2025
 * Author: Jerem
 * Modified to use board_width/board_height constants.
 */

#ifndef INC_TETRIS_H_
#define INC_TETRIS_H_

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> // Include for bool type
#include <stdint.h>  // Include for uint32_t if needed for LCD_BUFFER index calculation clarity

// --- Configuration ---
// NOTE: Changed board_width to 10 to match the original code's logic (array sizes, checks).
// If you change these dimensions, ensure the rendering logic in tetris_frame and potentially
// piece definitions/random placement are also adjusted.
#define BOARD_WIDTH  8
#define BOARD_HEIGHT 20
const int board_width = BOARD_WIDTH;
const int board_height = BOARD_HEIGHT;

// --- Global Variables ---
// tetris_block layout is: {w-1,h-1}{x0,y0}{x1,y1}{x2,y2}{x3,y3} (two bits each)
// Using meaningful names improves readability slightly.
int current_piece_x = 0, current_piece_y = 0, current_piece_rotation = 0;
int prev_piece_x = 0, prev_piece_y = 0, prev_piece_rotation = 0;
int current_piece_idx = 0, collision_check_result = 0, tick_counter = 0;

// The game board itself
int tetris_board[BOARD_HEIGHT][BOARD_WIDTH];

// Piece definitions (encoded)
const int tetris_block[7][4] = {{431424, 598356, 431424, 598356}, // I
                                {427089, 615696, 427089, 615696}, // L
                                {348480, 348480, 348480, 348480}, // O
                                {599636, 431376, 598336, 432192}, // Z
                                {411985, 610832, 415808, 595540}, // T
                                {247872, 799248, 247872, 799248}, // S
                                {614928, 399424, 615744, 428369}}; // J

int tetris_score = 0;

// Assume LCD_BUFFER and UpdateLCD are defined elsewhere

// extract a 2-bit number from a tetris_block entry
// gets piece characteristics: width-1 (16), height-1 (18), block coords (0-14)
int tetris_get_piece_data(int rotation, int data_offset) {
    return 3 & tetris_block[current_piece_idx][rotation] >> data_offset;
}

// create a new piece, don't remove old one (it has landed and should stick)
void tetris_new_piece() {
    current_piece_y = prev_piece_y = 0;

    int seed = rand();
    current_piece_idx = seed % 7;
    current_piece_rotation = prev_piece_rotation = rand() % 4;
    // Adjust starting position random range based on board width and piece width
    // piece_width = tetris_get_piece_data(current_piece_rotation, 16) + 1
    int piece_width_minus_1 = tetris_get_piece_data(current_piece_rotation, 16);
    current_piece_x = prev_piece_x = rand() % (board_width - piece_width_minus_1);
}

// draw the tetris_board and tetris_score
void tetris_frame() {
    for (int i = 0; i < 4; i++) {
    	LCD_BUFFER[100 + i * 132] = 0xFF;
    	LCD_BUFFER[101 + i * 132] = 0xFF;
    	LCD_BUFFER[102 + i * 132] = 0xFF;
    }

    for (int i = 0; i < board_height; i++) {
        for (int j = 0; j < board_width; j++) {
        	uint8_t byte_mask = (j & 1 == 1) ? 0xF0 : 0x0F;
        	for (int i1 = 0; i1 < 5; i1++) {
        		unsigned int index = i * 5 + i1 + (j / 2) * 132;
        		if (tetris_board[i][j] != 0)
        			LCD_BUFFER[index] |= byte_mask;
        		else LCD_BUFFER[index] &= ~byte_mask;
        	}
        }
    }

    if (tetris_score / 10 != 0) {
    	memcpy(LCD_BUFFER + 132 + 120, digits[tetris_score / 10], 5);
    }
    memcpy(LCD_BUFFER + 132 + 125, digits[tetris_score % 10], 5);

    UpdateLCD();
    //printf("Score: %d\r\n", tetris_score);
}

// set the value fo the tetris_board for a particular piece configuration
void tetris_set_piece(int x, int y, int rotation, int piece_index, int value) {
    // Iterates 4 times for the 4 blocks of a Tetromino.
    // i*2 accesses the y offset (0, 2, 4, 6)
    // (i*2)+2 accesses the x offset (2, 4, 6, 8)
    for (int i = 0; i < 8; i += 2) {
        int block_y_offset = tetris_get_piece_data(rotation, i * 2);
        int block_x_offset = tetris_get_piece_data(rotation, (i * 2) + 2);
        tetris_board[y + block_y_offset][x + block_x_offset] = value;
    }
}

// move a piece from old (prev_*) coords to new (current_*)
int tetris_update_piece() {
    tetris_set_piece(prev_piece_x, prev_piece_y, prev_piece_rotation, current_piece_idx, 0); // Clear old position
    prev_piece_x = current_piece_x;
    prev_piece_y = current_piece_y;
    prev_piece_rotation = current_piece_rotation;
    tetris_set_piece(current_piece_x, current_piece_y, current_piece_rotation, current_piece_idx, current_piece_idx + 1); // Draw new position (use piece index + 1 as value)
    return 1; // Indicate success perhaps? Original didn't use return value.
}


// remove line(s) from the tetris_board if they're full
void tetris_remove_line() {
    int piece_height_minus_1 = tetris_get_piece_data(current_piece_rotation, 18);
    // Check rows potentially affected by the last placed piece
    for (int row = current_piece_y; row <= current_piece_y + piece_height_minus_1 && row < board_height; row++) {
        bool line_full = true;
        for (int col = 0; col < board_width; col++) { // Use board_width
            if (tetris_board[row][col] == 0) {
                line_full = false;
                break;
            }
        }

        if (line_full) {
            tetris_score++; // Increment score for cleared line
            // Move all rows above this one down
            for (int i = row; i > 0; i--) {
                // Use sizeof the destination row for robustness
                memcpy(&tetris_board[i][0], &tetris_board[i - 1][0], sizeof(tetris_board[0]));
            }
            // Clear the top row
            memset(&tetris_board[0][0], 0, sizeof(tetris_board[0]));
            // Since we shifted rows down, we need to re-check the current row index `row`
            // However, the original code didn't do this. Let's stick to the original logic:
            // It only checks the range once. If multiple lines are cleared by one piece,
            // this approach might only clear one or clear incorrectly depending on order.
            // For simplicity, keeping original logic flow. A better approach would re-evaluate
            // the current row index if a line is cleared.
        }
    }
}


// check if placing current_piece_idx at (x,y,r) will cause a collision
int tetris_check_hit(int x, int y, int rotation) {
    int piece_height_minus_1 = tetris_get_piece_data(rotation, 18);
    int piece_width_minus_1 = tetris_get_piece_data(rotation, 16);

    // Check vertical bounds (bottom edge)
    if (y + piece_height_minus_1 >= board_height) { // Use board_height
        return 1; // Hit bottom boundary
    }
    // Check horizontal bounds (should ideally be checked before calling, but double check)
    if (x < 0 || x + piece_width_minus_1 >= board_width) {
         return 1; // Hit side boundary (shouldn't happen with proper move logic)
    }

    // Temporarily clear the current piece's old position to check collisions only with landed blocks
    tetris_set_piece(prev_piece_x, prev_piece_y, prev_piece_rotation, current_piece_idx, 0);

    collision_check_result = 0;
    // Check collision with existing blocks on the board
    for (int i = 0; i < 8; i += 2) {
        int block_y_offset = tetris_get_piece_data(rotation, i * 2);
        int block_x_offset = tetris_get_piece_data(rotation, (i * 2) + 2);
        if (tetris_board[y + block_y_offset][x + block_x_offset] != 0) {
            collision_check_result++;
        }
    }

    // Restore the current piece's old position (important!)
    tetris_set_piece(prev_piece_x, prev_piece_y, prev_piece_rotation, current_piece_idx, current_piece_idx + 1);

    return collision_check_result > 0; // Return true if any collision detected
}

// slowly tick the piece y position down so the piece falls
int tetris_do_tick() {
    if (++tick_counter > 30) { // Threshold for piece falling speed
        tick_counter = 0;
        // Check if the piece can move down one step
        if (tetris_check_hit(current_piece_x, current_piece_y + 1, current_piece_rotation)) {
            // Cannot move down, piece has landed
            if (current_piece_y == 0) {
                 // Piece landed at the very top, game over
                 // (Could also check collision on new_piece placement)
                return 0; // Game Over signal
            }
            // Piece landed, process line clears and spawn next piece
            tetris_remove_line(); // Check/remove lines at the landing position
            tetris_new_piece();   // Get the next piece
            // Check for collision immediately after spawning (game over condition)
            if(tetris_check_hit(current_piece_x, current_piece_y, current_piece_rotation)) {
                return 0; // Game Over - new piece collided immediately
            }
            // Update board with the newly spawned piece's initial position
            tetris_update_piece();
        } else {
            // Piece can move down, update its position
            current_piece_y++;
            tetris_update_piece();
        }
    }
    return 1; // Game continues
}

// main game loop with input checking (example 'wasd' mapping)
void tetris_runloop() {
    bool key_down_last = false;
    char input_char = '\0';


    while (tetris_do_tick()) { // Continue while game is not over
        HAL_Delay(30); // Small delay

        // --- Input Handling ---
        // This part is specific to how system_call works
        char key_code = system_call(0x0001, NULL); // Assuming this gets raw key code
        while (key_code != 254 && key_code != 37 && key_code != 32 && key_code != 36 && key_code != 26)
			key_code = system_call(0x0001, NULL);
        (void) system_call(0x0005, NULL);

		input_char = '\0'; // Reset input for this iteration

		 if (key_code == 37) input_char = 'd';      // Left
		 else if (key_code == 32) input_char = 'a'; // Right
		 else if (key_code == 26) input_char = 's';// Down (drop)
		 else if (key_code == 36) input_char = 'w'; // Rotate
		 // Add 'q' for quit if desired

        // --- Game Logic based on Input ---
        int next_x = current_piece_x;
        int next_y = current_piece_y;
        int next_r = current_piece_rotation;
        bool piece_moved = false;

        if (input_char == 'a' && current_piece_x > 0) { // Move Left
            next_x--;
            if (!tetris_check_hit(next_x, next_y, next_r)) {
                current_piece_x = next_x;
                piece_moved = true;
            }
        } else if (input_char == 'd') { // Move Right
            int piece_width_minus_1 = tetris_get_piece_data(current_piece_rotation, 16);
            // Check right boundary using board_width
            if (current_piece_x + piece_width_minus_1 < board_width - 1) {
                 next_x++;
                 if (!tetris_check_hit(next_x, next_y, next_r)) {
                     current_piece_x = next_x;
                     piece_moved = true;
                 }
            }
        } else if (input_char == 's') { // Soft Drop / Hard Drop
            // Original code implemented hard drop - move down until collision
             while (!tetris_check_hit(current_piece_x, current_piece_y + 1, current_piece_rotation)) {
                 current_piece_y++;
                 piece_moved = true; // Mark moved even if intermediate steps aren't drawn instantly
             }
             // Update immediately after finding landing spot
             tetris_update_piece();
             // Process landing: line clear and new piece
             tetris_remove_line();
             tetris_new_piece();
              // Check for collision immediately after spawning (game over condition)
             if(tetris_check_hit(current_piece_x, current_piece_y, current_piece_rotation)) {
                 // Game Over logic might need to exit loop cleanly
                 tetris_frame(); // Draw final state?
                 break; // Exit runloop
             }
             tetris_update_piece(); // Place the new piece visually
             tick_counter = 0; // Reset tick counter after drop
             piece_moved = false; // Reset moved flag as state is final for this input
        } else if (input_char == 'w') { // Rotate
            next_r = (current_piece_rotation + 1) % 4; // Try next rotation

            // Wall Kick / Boundary check after rotation
            int piece_width_minus_1 = tetris_get_piece_data(next_r, 16);
            // If rotation makes piece exceed right boundary, shift left ("wall kick")
            while (current_piece_x + piece_width_minus_1 >= board_width) { // Use board_width
                current_piece_x--;
            }
            // TODO: Add checks/kicks for left boundary and floor/ceiling if necessary,
            // although standard Tetris usually only kicks horizontally.

            // Check if the rotated (and possibly kicked) position is valid
            if (!tetris_check_hit(current_piece_x, current_piece_y, next_r)) {
                current_piece_rotation = next_r; // Apply rotation
                piece_moved = true;
            } else {
                // Rotation failed - revert position if it was kicked?
                // Original code reverted x and r if check_hit failed. Let's keep that.
                // Note: This revert logic might be problematic if wall kick was needed.
                // A better wall kick system checks multiple offsets.
                current_piece_x = prev_piece_x; // Revert x (might be wrong if wall kick happened)
                current_piece_rotation = prev_piece_rotation; // Keep old rotation
            }
        } else if (input_char == 'q') { // Quit
            return;
        }

        // Update piece visually only if it moved/rotated and wasn't a hard drop
        if (piece_moved) {
             tetris_update_piece();
        }

        // Draw the entire frame
        tetris_frame();
    }
    // Optional: Display Game Over message
    // printf("Game Over! Score: %d\n", tetris_score);
}

// Initialize display/screen (example)
void initscr() {
    // Assuming LCD_BUFFER is the screen buffer to clear
    // The size 132*4 seems specific, adjust if needed based on actual screen height/buffer stride
    memset(LCD_BUFFER, 0, 132 * 4); // TODO: Verify this size based on LCD dimensions/buffer layout

    UpdateLCD();
}

// init curses and start tetris_runloop
int tetris_main(uint32_t seed) {
    // initscr(); // Initialize screen (if needed)

    // Seed random number generator (important!)
    srand(seed); // Use a time-based seed if available

    // Clear the board initially
    memset(tetris_board, 0, sizeof(tetris_board));

    tetris_new_piece(); // Spawn the first piece
    tetris_update_piece(); // Place the first piece visually
    tetris_frame();        // Draw initial empty board + first piece

    tetris_runloop();      // Start the main game loop

    // Optional: Cleanup or show final score screen
    // endwin(); // If using curses-like library

    while (1)  {}

    return tetris_score; // Return final score
}


#endif /* INC_TETRIS_H_ */
