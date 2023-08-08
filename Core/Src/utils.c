/*
 * utils.c
 *
 *  Created on: Aug 4, 2023
 *      Author: phamv
 */

#include "utils.h"

uint8_t BCD2DEC(uint8_t data) {
	return (data >> 4) * 10 + (data & 0x0f);
}

uint8_t DEC2BCD(uint8_t data) {
	return (data / 10) << 4 | (data % 10);
}
