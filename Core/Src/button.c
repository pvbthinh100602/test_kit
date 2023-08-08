/*
 * button.c
 *
 *  Created on: Aug 2, 2023
 *      Author: phamv
 */

#include "button.h"
#include <stdio.h>

unsigned char button_count[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char spi_button = 0x00;

void button_init(){
	HAL_SPI_Init(&hspi1);
	HAL_GPIO_WritePin(BTN_LOAD_GPIO_Port, BTN_LOAD_Pin, 1);
}

void button_scan(){
	  HAL_GPIO_WritePin(BTN_LOAD_GPIO_Port, BTN_LOAD_Pin, 0);
	  HAL_GPIO_WritePin(BTN_LOAD_GPIO_Port, BTN_LOAD_Pin, 1);
	  HAL_SPI_Receive(&hspi1, &spi_button, 1, 1);
	  unsigned char mask = 0x80;
	  for(int i = 0; i < 8; i++){
		  if(mask > 0x0f){
			  if(spi_button & mask) button_count[3-i] = 0;
			  else button_count[3-i]++;
		  } else {
			  if(spi_button & mask) button_count[i] = 0;
			  else button_count[i]++;
		  }
		  mask = mask >> 1;
	  }
}

void button_test(){
	for(int i = 0; i < 8; i++){
		if(button_count[i] > 0){
			HAL_UART_Transmit(&huart1, (void*)str, sprintf(str, "Button %d pressed.\n", i+1), 100);
		}
	}
}
