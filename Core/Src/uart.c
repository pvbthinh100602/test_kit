/*
 * uart.c
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#include "uart.h"

unsigned char receive_buffer1 = 0;
unsigned char receive_buffer2 = 0;

void UART_Init(){
	HAL_UART_Receive_IT(&huart1, &receive_buffer1, 1);
	HAL_UART_Receive_IT(&huart2, &receive_buffer2, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		HAL_UART_Transmit(&huart1, &receive_buffer1, 1, 10);
		HAL_UART_Receive_IT(&huart1, &receive_buffer1, 1);
	}
	if(huart->Instance == USART2){
		fsm_GetTime();
		HAL_UART_Receive_IT(&huart2, &receive_buffer2, 1);
		HAL_UART_Transmit(&huart1, &receive_buffer2, 1, 10);
	}
}
