/*
 * ds3231.h
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#ifndef INC_DS3231_H_
#define INC_DS3231_H_

#include "global.h"
#include "uart.h"

extern uint8_t revBuffer[7];
extern uint8_t tranBuffer[7];

void rtc_init();
void rtc_fsm_get_time();

void rtc_Display7Seg();
void rtc_Read();
void rtc_UpdateTime();

uint8_t rtc_GetHour();
uint8_t rtc_GetMin();
uint8_t rtc_GetSec();
#endif /* INC_DS3231_H_ */
