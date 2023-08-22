/*
 * led7.h
 *
 *  Created on: Aug 2, 2023
 *      Author: phamv
 */

#ifndef INC_LED7_H_
#define INC_LED7_H_

#include "global.h"

//extern unsigned char led7[4];

void led7_init();

void led7_Scan();

void led7_SetDigit(int num, int position);
#endif /* INC_LED7_H_ */
