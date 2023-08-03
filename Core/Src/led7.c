/*
 * led7.c
 *
 *  Created on: Aug 2, 2023
 *      Author: phamv
 */

#include "led7.h"
#include "global.h"

unsigned char led7[4] = {0x00, 0xf1, 0x0e, 0x6a};

int led7_index = 0;

void led7Scan(){
	HAL_GPIO_WritePin(LD_LATCH_GPIO_Port, LD_LATCH_Pin, 0);
	HAL_SPI_Transmit(&hspi1, led7 + led7_index, 1, 1);
	HAL_GPIO_WritePin(LD_LATCH_GPIO_Port, LD_LATCH_Pin, 1);
	switch(led7_index){
	case 0:
		  HAL_GPIO_WritePin(LD_LED1_GPIO_Port, LD_LED1_Pin, 0);
		  HAL_GPIO_WritePin(LD_LED2_GPIO_Port, LD_LED2_Pin, 1);
		  HAL_GPIO_WritePin(LD_LED3_GPIO_Port, LD_LED3_Pin, 1);
		  HAL_GPIO_WritePin(LD_LED4_GPIO_Port, LD_LED4_Pin, 1);
		  break;
	case 1:
		  HAL_GPIO_WritePin(LD_LED1_GPIO_Port, LD_LED1_Pin, 1);
		  HAL_GPIO_WritePin(LD_LED2_GPIO_Port, LD_LED2_Pin, 0);
		  HAL_GPIO_WritePin(LD_LED3_GPIO_Port, LD_LED3_Pin, 1);
		  HAL_GPIO_WritePin(LD_LED4_GPIO_Port, LD_LED4_Pin, 1);
		  break;
	case 2:
		  HAL_GPIO_WritePin(LD_LED1_GPIO_Port, LD_LED1_Pin, 1);
		  HAL_GPIO_WritePin(LD_LED2_GPIO_Port, LD_LED2_Pin, 1);
		  HAL_GPIO_WritePin(LD_LED3_GPIO_Port, LD_LED3_Pin, 0);
		  HAL_GPIO_WritePin(LD_LED4_GPIO_Port, LD_LED4_Pin, 1);
		  break;
	case 3:
		  HAL_GPIO_WritePin(LD_LED1_GPIO_Port, LD_LED1_Pin, 1);
		  HAL_GPIO_WritePin(LD_LED2_GPIO_Port, LD_LED2_Pin, 1);
		  HAL_GPIO_WritePin(LD_LED3_GPIO_Port, LD_LED3_Pin, 1);
		  HAL_GPIO_WritePin(LD_LED4_GPIO_Port, LD_LED4_Pin, 0);
		  break;
	}

	led7_index = (led7_index + 1)%4;
}

void led7Set1Digit(int num, int position){
	if(num > 9 || num < 0) return;
	unsigned char digit = 0;
	switch(num){
	case 0:
		digit = 0x03;
		break;
	case 1:
		digit = 0x9e;
		break;
	case 2:
		digit = 0x25;
		break;
	case 3:
		digit = 0x0d;
		break;
	case 4:
		digit = 0x99;
		break;
	case 5:
		digit = 0x49;
		break;
	case 6:
		digit = 0x41;
		break;
	case 7:
		digit = 0x1f;
		break;
	case 8:
		digit = 0x01;
		break;
	case 9:
		digit = 0x09;
		break;
	}
	led7[position - 1] = digit;
}

//void led7Test(){
//	led7[0] = (led7[0] + 1)%10;
//	led7[1] = (led7[1] + 2)%10;
//	led7[2] = (led7[2] + 3)%10;
//	led7[3] = (led7[3] + 4)%10;
//}
