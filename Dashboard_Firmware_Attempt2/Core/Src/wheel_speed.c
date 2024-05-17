/*
 * wheel_speed.c
 *
 *  Created on: Apr 17, 2024
 *      Author: leoja
 */


#include "wheel_speed.h"
#define TIM_CLK 108000000

void WheelSpeed_Init(WheelSpeed_t* ws, TIM_HandleTypeDef* h_tim)
{
	ws->h_tim = h_tim;
	ws->last_count = 0;
	ws->last_tick = HAL_GetTick();
	HAL_TIM_Base_Start(h_tim);
}

uint32_t WheelSpeed_GetCPS(WheelSpeed_t* ws)
{
	uint32_t tick = HAL_GetTick();
	uint32_t count = __HAL_TIM_GET_COUNTER(ws->h_tim);

	uint32_t diff = tick - ws->last_tick;
	if(diff == 0) diff = 1; // prevent divide by 0

	uint32_t cps = 1000 * (count - ws->last_count) / diff;

	ws->last_count = count;
	ws->last_tick = tick;

	return cps;
}

void WheelSpeedPW_Init(WheelSpeedPW_t* ws, TIM_HandleTypeDef* h_tim, uint32_t tim_channel){
	ws->h_tim = h_tim;
	ws->tim_channel = tim_channel;
	HAL_TIM_Base_Start(h_tim);
	HAL_TIM_IC_Start_DMA(h_tim, tim_channel, ws->buf, 1);
}

uint32_t WheelSpeedPW_GetCPS(WheelSpeedPW_t* ws){
	if(ws->buf[0] == 0) return 0;
	uint32_t clk = TIM_CLK / ws->h_tim->Init.Prescaler;

	uint32_t ccr = ws->buf[0];
	uint32_t counter = __HAL_TIM_GET_COUNTER(ws->h_tim);
	if(counter > ccr) ccr = counter;

	uint32_t cps = clk / ccr;
	return cps;
}
