/*
 * profiler.c
 *
 *  Created on: Jan 14, 2025
 *      Author: IanKM
 */

#include "profiler.h"

#include "stm32f7xx_hal.h"
#include "semphr.h"

extern UART_HandleTypeDef huart3;

static SemaphoreHandle_t profiler_mutex = 0;
static StaticSemaphore_t profiler_mutex_buffer;

void profiler_init(void)
{
	profiler_mutex = xSemaphoreCreateMutexStatic(&profiler_mutex_buffer);
}

void profiler_record_marker(profiler_data_t* marker)
{
	marker->end = profiler_perf_timer;

	xSemaphoreTake(profiler_mutex, portMAX_DELAY);
	HAL_UART_Transmit(&huart3, (const uint8_t*)marker, sizeof(profiler_data_t), 0xFA57);
	xSemaphoreGive(profiler_mutex);
}
