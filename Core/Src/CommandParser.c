/*
 * CommandParser.c
 *
 *  Created on: Jul 13, 2025
 *      Author: Jerem
 */


#include "CommandParser.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "fatfs.h"
#include "usbd_cdc_acm_if.h" // to turn on/off echo

struct CLICommand** _cli_commands = 0;
uint16_t _cli_commands_capacity = 0;
uint16_t _cli_commands_size = 0;

uint8_t (*readfill_stdin)(char*, uint32_t) = 0;
uint8_t (*read_stdin)(char*, uint32_t) = 0;

// attempts to expand the _cli_commands array. If it fails, then the _cli_commands array will remain unchanged.
// if _cli_commands == null, then a new array will be allocated
void _grow(uint16_t new_size) {
	// if it has already been allocated before, then just reallocate it
	if (_cli_commands_capacity != 0) {
		struct CLICommand** new_arr = (struct CLICommand**) realloc(_cli_commands, sizeof(struct CLICommand*) * new_size);
		// if new_arr is null, then it failed to grow
		if (new_arr != 0) {
			_cli_commands = new_arr;
			_cli_commands_capacity = new_size;
		}

		return;
	}

	_cli_commands = malloc(sizeof(struct CLICommand*) * new_size);
	if (_cli_commands != 0) _cli_commands_capacity = new_size;
}

void hint_total_commands(uint16_t total_commands) {
	_grow(total_commands);

	if (_cli_commands_capacity < total_commands) return;
}

void register_cli_command(struct CLICommand* command) {
	if (_cli_commands_size == _cli_commands_capacity) _grow(_cli_commands_capacity + 1);
	if (_cli_commands_size >= _cli_commands_capacity) return;

	_cli_commands[_cli_commands_size++] = command;
}

struct CLICommand* _lookup_command(const char* name_start, const char* name_end) {
	unsigned int length = name_end - name_start;

	for (unsigned int i = 0; i < _cli_commands_size; i++) {
		if (length == strlen(_cli_commands[i]->function_name) && memcmp(_cli_commands[i]->function_name, name_start, length) == 0) return _cli_commands[i];
	}
	return NULL;
}

/**
 * Parses a command line
 * regular expression: " *([a-zA-Z0\-\_+]) *( +[.+])?"
 *
 * Returns
 * 0: success
 * 1: Empty command line
 * 2: Command not found
 */
uint8_t parse_command(const char* command_line, struct CLICall* call_obj) {
	const char* command_start = command_line;
	// first check for starting spaces
	while (*command_start == ' ' && *command_start != '\0') command_start++;
	if (*command_start == '\0') {
		call_obj = NULL;
		return 1; // no command found
	}

	const char* command_end = command_start + 1; // search for space at end of command name
	while (*command_end != ' ' && *command_end != '\0') command_end++;

	// search for start of arguments
	const char* arg_start = command_end;
	while (*arg_start == ' ' && *arg_start != '\0') arg_start++;

	struct CLICommand* command = _lookup_command(command_start, command_end);

	if (command == NULL) { // command not found
		call_obj = NULL;
		return 2;
	}

	call_obj->command = command;
	call_obj->arguments = arg_start;

	return 0;
}

/*
 * 		if (hUsbDeviceFS->dev_state == USBD_STATE_CONFIGURED) {
			char* current_directory = command;
			FRESULT success = f_getcwd(command, sizeof(command));

			if (success != FR_OK) {
				fputs("System error\n", stderr);
			}

			printf("\e[0;32m%s:%s%s$\e[0m ", systemConfigData.device_name, current_directory, (current_menu != FLASH_MENU) ? "" : "~frw");
			fflush(stdout);
		}
 */
uint8_t command_buffer[256];
void exec_shell() {
	FRESULT result = f_getcwd(command_buffer, 256);
	if (result != FR_OK) Error_Handler();

	if (echo_on) {
		printf("\e[0;32m%s:%s$\e[0m ", systemConfigData.device_name, command_buffer);
		fflush(stdout);
	}

	read_stdin(command_buffer, 256);
	if (command_buffer[0] == '\0') return;

	struct CLICall call;

	if (parse_command(command_buffer, &call) == 0)
		call.command->function(call.arguments);
	else fputs("Command not found\n", stderr);

	fflush(stdout);
}
