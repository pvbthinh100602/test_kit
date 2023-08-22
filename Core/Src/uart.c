/*
 * uart.c
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#include "uart.h"

unsigned char receive_buffer1 = 0;
unsigned char receive_buffer2 = 0;

uint8_t msg[100];

void uart_init(){
	HAL_UART_Receive_IT(&huart1, &receive_buffer1, 1);
	HAL_UART_Receive_IT(&huart2, &receive_buffer2, 1);
}

void uart_SendString(UART_HandleTypeDef *huart, uint8_t* str){
	HAL_UART_Transmit(huart, (void*)msg, sprintf(msg,"%s",str), 10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		HAL_UART_Transmit(&huart1, &receive_buffer1, 1, 10);
		HAL_UART_Receive_IT(&huart1, &receive_buffer1, 1);
	}
	if(huart->Instance == USART2){
		rtc_fsm_get_time();
		HAL_UART_Receive_IT(&huart2, &receive_buffer2, 1);
		HAL_UART_Transmit(&huart1, &receive_buffer2, 1, 10);
	}
}


