/*
 * pwm.c
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#include "pwm.h"

unsigned char pwm = 0;

void pwm_init(){
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}

void pwm_SetDutyCycle(unsigned char value){
	__HAL_TIM_SET_COMPARE(&htim13,TIM_CHANNEL_1,value);
}

void pwm_Test(){
	if(button_count[0] == 1){
		if(pwm == 0) pwm = 5;
		else if(pwm == 5) pwm = 9;
		else pwm = 0;
	}
	pwm_SetDutyCycle(pwm);
}
