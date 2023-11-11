/*
 * can_manager.c
 *
 *  Created on: Nov 8, 2023
 *      Author: justinchang
 */
#include "can_manager.h"

extern volatile uint16_t torque_limit;

void can_send_switches(CAN_HandleTypeDef hcan, uint8_t switches) {
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8] = {0};
	uint32_t TxMailbox;

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 2;
	//TxHeader.StdId = TODO:Find ID;
	TxHeader.RTR = CAN_RTR_DATA;

	TxData[0] = switches;
	TxData[1] = torque_limit && 0xFF;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

