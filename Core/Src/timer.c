/*
 * timer.c
 *
 *  Created on: Aug 1, 2023
 *      Author: phamv
 */
#include "timer.h"

int flag_timer1 = 0, flag_timer2 = 0, flag_timer3 = 0;
int timer1_counter = 0, timer2_counter = 0, timer3_counter = 0;

void setTimer1(int duration){
	timer1_counter = 1000;
	flag_timer1 = 0;
}

void setTimer2(int duration){
	timer2_counter = duration/TIMER_CYCLE;
	flag_timer2 = 0;
}

void setTimer3(int duration){
	timer3_counter = duration/TIMER_CYCLE;
	flag_timer3 = 0;
}


void timerRun(){
	if(timer1_counter > 0){
		timer1_counter--;
		if(timer1_counter == 0) flag_timer1 = 1;
	}

	if(timer2_counter > 0){
		timer2_counter--;
		if(timer2_counter == 0) flag_timer2 = 1;
	}
	if(timer3_counter > 0){
		timer3_counter--;
		if(timer3_counter == 0) flag_timer3 = 1;
	}
}
