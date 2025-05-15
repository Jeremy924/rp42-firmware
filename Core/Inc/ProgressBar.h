/*
 * ProgressBar.h
 *
 *  Created on: May 2, 2025
 *      Author: Jerem
 */

#ifndef INC_PROGRESSBAR_H_
#define INC_PROGRESSBAR_H_
#include "spi.h"

#include "main.h"

void init_progress_bar();

void set_progress(uint8_t progress);
#define CONTINUOUS_BAR_LENGTH 25
void continuous_progres_bar(uint8_t* last);
#endif /* INC_PROGRESSBAR_H_ */
