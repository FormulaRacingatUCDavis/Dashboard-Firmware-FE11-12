/*
 * can_manager.c
 *
 *  Created on: Feb 20, 2024
 *      Author: cogus
 */
#include "can_manager.h"
#include <stdio.h>

extern volatile state_t state;
extern volatile error_t error;

extern volatile uint16_t TC_torque_adjustment;

volatile uint8_t mc_lockout;
volatile uint8_t mc_enabled;
volatile uint16_t capacitor_volt = 0;
volatile uint8_t shutdown_flags = 0b00111000;  //start with shutdown flags OK
volatile uint8_t estop_flags = 0;
volatile uint8_t switches = 0xC0;   //start with switches on to stay in startup state
volatile uint8_t PACK_TEMP;
volatile uint8_t mc_fault;

// From TCAN
volatile uint16_t front_right_wheel_speed = 0;
volatile uint16_t front_left_wheel_speed = 0;
volatile uint16_t back_right_wheel_speed = 0;
volatile uint16_t back_left_wheel_speed = 0;

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;
uint8_t               RxData[8];

CAN_HandleTypeDef hcan1;

// receive buffer message
//CAN_MSG_OBJ msg_RX;

/********** OUTGOING CAN MESSAGES **********/

// mc command
//CAN_MSG_FIELD field_TX_mc_command = {
//    .idType = 0,
//    .frameType = 0,
//    .dlc = 6,
//    .formatType = 0,
//    .brs = 0
//};
//conversion to new type of can below
void can_set_up() {
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = TORQUE_REQUEST;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 6;

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = VEHICLE_STATE;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;

}



// VCU state
//CAN_MSG_FIELD field_TX_vcu_state = {
//    .idType = 0,
//    .frameType = 0,
//    .dlc = 8,
//    .formatType = 0,
//    .brs = 0
//};


//CAN_MSG_OBJ msg_TX_vcu_state = {
//    .msgId = VEHICLE_STATE,
//    .field = {0}, // null
//    .data = 0 // null pointer
//};

/************ CAN ************/


void CAN_Receive() {
    // gets message and updates values
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, RxHeader, RxData)) {

    	if(RxHeader.StdId == BMS_STATUS_MSG) {
    		temp_attenuate();
    	} else if(RxHeader.StdId == PEI_CURRENT_SHUTDOWN) {
    		shutdown_flags = RxData[2];
    	} else {
    	}

//below is the code from the fe10 version
//        switch (msg_RX.msgId) {
//            case BMS_STATUS_MSG:
//                PACK_TEMP = msg_RX.data[0];
//                temp_attenuate();
//                break;
//            case PEI_CURRENT_SHUTDOWN:
//                shutdown_flags = msg_RX.data[2];
//                break;
//            default:
//                // no valid input received
//                break;
//        }
    	char str[80];
    	sprintf(str, "RxHeader = %lu", RxHeader.StdId);
    	CDC_Trasmit_FS(str, strlen(str));
        //printf("ID: %lu \n\r", RxHeader.StdId);
        for (uint8_t i = 1; i < RxHeader.DLC; i++) {
        	char str_data[80];
        	sprintf(str_data,"RxData = %d", RxData[i]);
        	CDC_Trasmit_FS(str_data, strlen(str_data));
            //printf("byte %d: %d\n\r", i, RxData[i]);
        }
    }
}



//  CAN transmit torque request command
void can_tx_vcu_state(){
        uint8_t data_TX_state[6] = {
        0,
        0,
        0,
        0,
        one_byte_state(),
        braking()
    };

    //adding vehicle state stuff to CAN header
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = VEHICLE_STATE;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
//    msg_TX_vcu_state.field = field_TX_vcu_state;
//    msg_TX_vcu_state.data = data_TX_state;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data_TX_state, &TxMailbox);
}

void can_tx_torque_request(){

    uint16_t throttle_msg_byte = 0;
    if (state == DRIVE) {
        throttle_msg_byte = requested_throttle() - TC_torque_adjustment;
    }

    uint8_t byte5 = 0b010;   //speed mode | discharge_enable | inverter enable
    byte5 |= (hv_requested() & 0x01);  //set inverter enable bit

    uint8_t data_TX_torque[8] = {
        (uint8_t)(throttle_msg_byte & 0xff), // 0 - torque command lower (Nm*10)
        (uint8_t)(throttle_msg_byte >> 8) & 0xFF, // 1 - torque command upper (Nm*10)
        0, // 2 - speed command lower (not applicable)
        0, // 3 - speed command upper (not applicable)
        1, // 4 - direction (1 = forward, 0 = backward)
        byte5, // 5 - speed mode | discharge_enable | inverter enable
        0, // 6 - torque limit lower (if 0, default EEPROM value used)
        0 // 7 - torque limit upper (if 0, default EEPROM value used)
    };

    //adding torque request stuff to CAN header
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = TORQUE_REQUEST;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 6;
//    msg_TX_mc_command.field = field_TX_mc_command;
//    msg_TX_mc_command.data = data_TX_torque;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data_TX_torque, &TxMailbox);
}


void can_tx_disable_MC() {
    uint8_t data_TX_disable_mc[] = { 0, 0, 0, 0, 0, 0, 0 };

    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = TORQUE_REQUEST;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 6;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data_TX_disable_mc, &TxMailbox);
}

