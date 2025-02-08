/*
 * profiler.c
 *
 *  Created on: Jan 14, 2025
 *      Author: IanKM
 */

#include "profiler.h"

#include "stm32f7xx_hal.h"

extern UART_HandleTypeDef huart3;

void profiler_record_marker(profiler_data_t* marker)
{
	marker->end = (uint64_t)HAL_GetTick();
	HAL_UART_Transmit(&huart3, (const uint8_t*)marker, sizeof(profiler_data_t), 0xFA57);
}
