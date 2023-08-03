/*
 * led7.h
 *
 *  Created on: Aug 2, 2023
 *      Author: phamv
 */

#ifndef INC_LED7_H_
#define INC_LED7_H_

#include "global.h"

extern unsigned char led7[4];

void led7Scan();
void led7Set1Digit(int num, int position);
//void led7Test();
void led7ColonEn(int enable);

#endif /* INC_LED7_H_ */
