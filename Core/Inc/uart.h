/*
 * uart.h
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"

extern unsigned char receive_buffer1, receive_buffer2;

void uart_init();
void uart_SendString(UART_HandleTypeDef *huart, uint8_t* str);

#endif /* INC_UART_H_ */
