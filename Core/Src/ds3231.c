/*
 * ds3231.c
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#include "ds3231.h"

#define DS3231_ADDRESS 0x68<<1

#define WAIT_START 0
#define GET_HOUR 1
#define GET_MINUTE 2
#define GET_SECOND 3

uint8_t revBuffer[7];
uint8_t tranBuffer[7];

unsigned char start = 0x02;
unsigned char status = WAIT_START;
unsigned char flag_finish = 0;
unsigned char value  = 0;

uint8_t read_count = 0;

typedef struct {
	uint8_t hours;
	uint8_t min;
	uint8_t sec;
	uint8_t date;
	uint8_t day;
	uint8_t month;
	uint8_t year;
} DS3231_typedef;

DS3231_typedef DS3231_TimeNow;
DS3231_typedef DS3231_TimeSet;

void rtc_UpdateTime(){
	HAL_UART_Transmit(&huart2, &start, 1, 10);
	while(!flag_finish);
	flag_finish = 0;
}

void rtc_fsm_get_time(){
	switch (status) {
		case WAIT_START:
			if(receive_buffer2 == '!') status = GET_HOUR;
			break;
		case GET_HOUR:
			if(receive_buffer2 == ':'){
				tranBuffer[2] = DEC2BCD(value);
				value = 0;
				status = GET_MINUTE;
			} else {
				value = value*10 + receive_buffer2 - '0';
			}
			break;
		case GET_MINUTE:
			if(receive_buffer2 == ':'){
				tranBuffer[1] = DEC2BCD(value);
				value = 0;
				status = GET_SECOND;
			} else {
				value = value*10 + receive_buffer2 - '0';
			}
			break;
		case GET_SECOND:
			if(receive_buffer2 == '#'){
				tranBuffer[0] = DEC2BCD(value);
				value = 0;
				flag_finish = 1;
				status = WAIT_START;
			} else {
				value = value*10 + receive_buffer2 - '0';
			}
			break;
		default:
			break;
	}
}



void rtc_init(){
//	tranBuffer[0] = DEC2BCD(00);
//	tranBuffer[1] = DEC2BCD(9);
//	tranBuffer[2] = DEC2BCD(10);
//	rtc_UpdateTime();
//	tranBuffer[3] = DEC2BCD(5);
//	tranBuffer[4] = DEC2BCD(3);
//	tranBuffer[5] = DEC2BCD(8);
//	tranBuffer[6] = DEC2BCD(23);
//	HAL_I2C_Mem_Write_IT(&hi2c1, DS3231_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, tranBuffer, 7);
}

void rtc_Display7Seg(){
	  led7_SetDigit(DS3231_TimeNow.hours/10, 1);
	  led7_SetDigit(DS3231_TimeNow.hours%10, 2);
	  led7_SetDigit(DS3231_TimeNow.min/10, 3);
	  led7_SetDigit(DS3231_TimeNow.min%10, 4);
}

void rtc_Read(){
	read_count = (read_count + 1)%20;
	if(read_count == 0){
		HAL_I2C_Mem_Read_IT(&hi2c1, DS3231_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, revBuffer, 7);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	UNUSED(hi2c);
	if (hi2c->Instance == I2C1) {
		DS3231_TimeNow.sec = BCD2DEC(revBuffer[0]);
		DS3231_TimeNow.min = BCD2DEC(revBuffer[1]);
		DS3231_TimeNow.hours = BCD2DEC(revBuffer[2]);
		DS3231_TimeNow.day = BCD2DEC(revBuffer[3]);
		DS3231_TimeNow.date = BCD2DEC(revBuffer[4]);
		DS3231_TimeNow.month = BCD2DEC(revBuffer[5]);
		DS3231_TimeNow.year = BCD2DEC(revBuffer[6]);
	}
}

uint8_t rtc_GetHour(){
	return DS3231_TimeNow.hours;
}

uint8_t rtc_GetMin(){
	return DS3231_TimeNow.min;
}

uint8_t rtc_GetSec(){
	return DS3231_TimeNow.sec;
}
