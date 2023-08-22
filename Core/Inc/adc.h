/*
 * adc.h
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"
#include "uart.h"
#include <stdio.h>

void adc_init();

void adc_ReadSensor();

uint32_t adc_GetLight();

uint32_t adc_GetVarResistor();

float adc_GetVoltage();

float adc_GetCurrent();

void adc_Test();

#endif /* INC_ADC_H_ */
