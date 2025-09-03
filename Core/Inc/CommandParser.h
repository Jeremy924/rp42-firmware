/*
 * CommandParser.h
 *
 *  Created on: Jul 13, 2025
 *      Author: Jerem
 */

#ifndef SRC_COMMANDPARSER_H_
#define SRC_COMMANDPARSER_H_
#include <stdint.h>

struct CLICommand {
	char* function_name;
	void (*function)(char*);
};

struct CLICall {
	struct CLICommand* command;
	const char* arguments;
};

extern uint8_t (*readfill_stdin)(char*, uint32_t);
extern uint8_t (*read_stdin)(char*, uint32_t);

extern struct CLICommand** _cli_commands;
extern uint16_t _cli_commands_capacity;
extern uint16_t _cli_commmands_size;

extern void hint_total_commands(uint16_t);

extern void register_cli_command(struct CLICommand*);

extern uint8_t parse_command(const char* command_line, struct CLICall* call_obj);

void exec_shell();

#endif /* SRC_COMMANDPARSER_H_ */
