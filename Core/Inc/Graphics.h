/*
 * Graphics.h
 *
 *  Created on: Aug 2, 2025
 *      Author: Jerem
 */

#ifndef INC_GRAPHICS_H_
#define INC_GRAPHICS_H_


extern void show_notification(char* notification);

extern void hide_notification();

extern void show_menu(char* icons, unsigned int total_icons, unsigned int page);
extern void hide_menu();

extern int text_entry(char* result, unsigned int max_result, char* message);

#endif /* INC_GRAPHICS_H_ */
