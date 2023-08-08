/*
 * dht11.h
 *
 *  Created on: Aug 3, 2023
 *      Author: phamv
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "global.h"

uint8_t DHT_ReadTempHum();

float DHT_GetTemp();

float DHT_GetHumi();

#endif /* INC_DHT11_H_ */
