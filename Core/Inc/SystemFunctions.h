/*
 * SystemFunctions.h
 *
 *  Created on: Jul 14, 2025
 *      Author: Jerem
 */

#ifndef INC_SYSTEMFUNCTIONS_H_
#define INC_SYSTEMFUNCTIONS_H_

extern void clear_command_func(char*);
extern void echo_command_func(char*);
extern void bootmode_command_func(char*);

extern FRESULT update_system_config();
extern const char* fresult_to_string(FRESULT);

extern void register_all_sys_functions();

#endif /* INC_SYSTEMFUNCTIONS_H_ */
