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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		timerRun();
		led7Scan();
	}
}

