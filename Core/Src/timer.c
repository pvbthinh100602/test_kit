/*
 * timer.c
 *
 *  Created on: Aug 1, 2023
 *      Author: phamv
 */
#include "timer.h"

void timer_init(){
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Init(&htim3);
	HAL_TIM_Base_Start(&htim3);
//	HAL_TIM_PWM_Init(&htim13);
	setTimer1(1000);
	setTimer2(50);
}

int flag_timer1 = 0, flag_timer2 = 0, flag_timer3 = 0;
int timer1_counter = 0, timer2_counter = 0, timer3_counter = 0;
int timer1_MUL = 0, timer2_MUL = 0, timer3_MUL = 0;

void setTimer1(int duration){
	timer1_MUL = duration/TIMER_CYCLE;
	timer1_counter = timer1_MUL;
	flag_timer1 = 0;
}

void setTimer2(int duration){
	timer2_MUL = duration/TIMER_CYCLE;
	timer2_counter = timer2_MUL;
	flag_timer2 = 0;
}

void setTimer3(int duration){
	timer3_MUL = duration/TIMER_CYCLE;
	timer3_counter = timer3_MUL;
	flag_timer3 = 0;
}


void timerRun(){
	if(timer1_counter > 0){
		timer1_counter--;
		if(timer1_counter == 0) {
			flag_timer1 = 1;
			timer1_counter = timer1_MUL;
		}
	}

	if(timer3_counter > 0){
		timer3_counter--;
		if(timer3_counter == 0) {
			flag_timer3 = 1;
			timer3_counter = timer3_MUL;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		if(timer2_counter > 0){
			timer2_counter--;
			if(timer2_counter == 0) {
				flag_timer2 = 1;
				timer2_counter = timer2_MUL;
			}
		}
		timerRun();
		led7Scan();
	}
}

