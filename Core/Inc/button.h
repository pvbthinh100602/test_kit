/*
 * button.h
 *
 *  Created on: Aug 2, 2023
 *      Author: phamv
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "global.h"

extern unsigned char button_count[8];

void button_init();
void button_Scan();
void button_Test();

#endif /* INC_BUTTON_H_ */
