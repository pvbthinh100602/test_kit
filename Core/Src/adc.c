/*
 * adc.c
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#include "adc.h"
#include "lcd.h"

uint32_t adcReceive[4];

uint8_t counter_test = 0;

void adc_init(){
	HAL_ADC_Init(&hadc1);
}


void adc_ReadSensor(){
	HAL_ADC_Start_DMA(&hadc1, adcReceive, 4);
//	HAL_ADC_Stop_DMA(&hadc1);
}

uint32_t adc_GetLight(){
	return adcReceive[0];
}

uint32_t adc_GetVarResistor(){
	return adcReceive[1];
}

float adc_GetVoltage(){
	return ((float)adcReceive[2]*3.3*12)/(4095*1.565);
}

float adc_GetCurrent(){
	return (((float)adcReceive[3]*3.3)/(4095*0.647)-2.5)*5/2.5;
}

void adc_Test(){
	counter_test = (counter_test + 1)%20;
	if(counter_test == 0){
		adc_ReadSensor();
		char msg[100];
		HAL_UART_Transmit(&huart1, (void*)msg, sprintf(msg, "Light: %ld, VarResistor: %ld, Voltage: %.2f, Current: %.4f\n", adc_GetLight(), adc_GetVarResistor(), adc_GetVoltage(), adc_GetCurrent()), 10);
		sprintf(msg, "Vol: %.2fV, Cur: %.2fmA", adc_GetVoltage(), adc_GetCurrent()*1000);
		lcd_Fill(0, 100, lcddev.width, 120, BLACK);
		lcd_StrCenter(0,100,msg,WHITE,BLUE,16,1);
	}

}

