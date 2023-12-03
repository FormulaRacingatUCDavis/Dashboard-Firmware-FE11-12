/*
 * pin_read.c
 *
 *  Created on: Nov 11, 2023
 *      Author: justinchang
 */

#include <pin_read.h>
#include "main.h"

uint8_t HV_Read() {
	return HAL_GPIO_ReadPin(HV_READ_GPIO_Port, HV_READ_Pin);
}

uint8_t Drive_Read() {
	return HAL_GPIO_ReadPin(Drive_READ_GPIO_Port, Drive_READ_Pin);
}
