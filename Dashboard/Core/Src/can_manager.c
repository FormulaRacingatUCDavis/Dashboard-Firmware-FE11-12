/*
 * can_manager.c
 *
 *  Created on: Nov 8, 2023
 *      Author: justinchang
 */
#include "can_manager.h"

void can_send_switches(CAN_HandleTypeDef hcan, uint8_t switches) {
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];
	uint32_t TxMailbox;

	TxHeader.IDE = CAN_ID_STD;
	//Needs finishing

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

