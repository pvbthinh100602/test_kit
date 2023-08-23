/*
 * timer.h
 *
 *  Created on: Aug 1, 2023
 *      Author: phamv
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "global.h"

#define TIMER_CYCLE 1

extern int flag_timer1, flag_timer2, flag_timer3;

void setTimer1(int duration);
void setTimer2(int duration);
void setTimer3(int duration);
void timer_init();

#endif /* INC_TIMER_H_ */
