/*
 * profiler.c
 *
 *  Created on: Jan 14, 2025
 *      Author: IanKM
 */

#include "profiler.h"

extern UART_HandleTypeDef huart3;

void profiler_record_marker(profiler_data_t marker)
{
	marker.end = profiler_perf_timer;
	HAL_UART_Transmit(huart3, &marker, sizeof(marker), 0xFA57);
}
