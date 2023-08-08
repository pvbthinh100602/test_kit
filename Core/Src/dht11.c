/*
 * dht11.c
 *
 *  Created on: Aug 3, 2023
 *      Author: phamv
 */

#include "dht11.h"
#include <stdio.h>

#define START_TIME 18000

float DHT_Temp = 0.0;
float DHT_Humi = 0.0;

static void DHT_SetPinOut()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ONE_WIRE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ONE_WIRE_GPIO_Port, &GPIO_InitStruct);
}
static void DHT_SetPinIn()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ONE_WIRE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ONE_WIRE_GPIO_Port, &GPIO_InitStruct);
}

static uint8_t DHT_ReadPin()
{
	uint8_t Value;
	Value =  HAL_GPIO_ReadPin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin);
	return Value;
}

static uint8_t DHT_Start()
{
	uint8_t Response = 0;
	DHT_SetPinOut();
	HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, 0);
	delay_us(START_TIME);
	HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, 1);
	delay_us(40);
	DHT_SetPinIn();
	if (!DHT_ReadPin())
	{
		delay_us(80);
		if(DHT_ReadPin())
		{
			Response = 1;
		}
		else Response = 0;
	}

	while(DHT_ReadPin());
	return Response;
}

static uint8_t DHT_Read()
{
	uint8_t Value = 0;
//	DHT_SetPinIn();
	for(int i = 0; i<8; i++)
	{
		while(!DHT_ReadPin());
		delay_us(50);
		if(!DHT_ReadPin())
		{
			Value &= ~(1<<(7-i));
		}
		else Value |= 1<<(7-i);
		while(DHT_ReadPin());
	}
	return Value;
}

uint8_t DHT_ReadTempHum()
{
	uint8_t Temp1, Temp2, RH1, RH2;
	uint16_t Temp, Humi, SUM = 0;
	DHT_Start();
	RH1 = DHT_Read();
	RH2 = DHT_Read();
	Temp1 = DHT_Read();
	Temp2 = DHT_Read();
	SUM = DHT_Read();
	Temp = (Temp1<<8)|Temp2;
	Humi = (RH1<<8)|RH2;
	DHT_Temp = (float)(Temp/10.0);
	DHT_Humi = (float)(Humi/10.0);
	return SUM;
}

float DHT_GetTemp(){
	return DHT_Temp;
}

float DHT_GetHumi(){
	return DHT_Humi;
}
