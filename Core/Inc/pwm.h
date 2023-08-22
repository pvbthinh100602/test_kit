/*
 * pwm.h
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "global.h"

void pwm_init();
void pwm_Test();
void pwm_SetDutyCycle(unsigned char value);
#endif /* INC_PWM_H_ */
