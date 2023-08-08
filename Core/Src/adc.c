/*
 * adc.c
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#include "adc.h"

uint32_t adcReceive[4];

void ADC_Init(){
	HAL_ADC_Init(&hadc1);
}


void ADC_ReadSensor(){
	HAL_ADC_Start_DMA(&hadc1, adcReceive, 4);
//	HAL_ADC_Stop_DMA(&hadc1);
}

uint32_t ADC_GetLight(){
	return adcReceive[0];
}

uint32_t ADC_GetVarResistor(){
	return adcReceive[1];
}

float ADC_GetVoltage(){
	return ((float)adcReceive[2]*3.3*12)/(4095*1.565);
}

float ADC_GetCurrent(){
	return (((float)adcReceive[3]*3.235)/(4095*0.647)-2.5)*5/2.5;
}

void test_adc(){
	ADC_ReadSensor();
	char msg[100];
	HAL_UART_Transmit(&huart1, (void*)msg, sprintf(msg, "Light: %ld, VarResistor: %ld, Voltage: %.2f, Current: %.4f\n", ADC_GetLight(), ADC_GetVarResistor(), ADC_GetVoltage(), ADC_GetCurrent()), 10);

}

