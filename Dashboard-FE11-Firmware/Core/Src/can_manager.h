/*
 * can_manager.h
 *
 *  Created on: Feb 20, 2024
 *      Author: cogus
 */

#ifndef SRC_CAN_MANAGER_H_
#define SRC_CAN_MANAGER_H_

#include "stdint.h"

#include "stm32f7xx_hal.h"
#include "fsm.h"
#include "sensors.h"


/********** ENUM OF CAN IDS **********/
typedef enum {
    // PCAN
    //BSPD_FLAGS = 0x0c1,
    //DRIVER_SWITCHES = 0x050,
    VEHICLE_STATE = 0x766,
    TORQUE_REQUEST = 0x0C0,
    BMS_STATUS_MSG = 0x380,
    PEI_CURRENT_SHUTDOWN = 0x387,
    //MC_VOLTAGE_INFO = 0x0A7,
    //MC_INTERNAL_STATES = 0xAA,
    //MC_FAULT_CODES = 0xAB,
    // TCAN
//    FRONT_LEFT_WHEEL_SPEED = 0x470,
//    FRONT_RIGHT_WHEEL_SPEED = 0x471,
//    BACK_LEFT_WHEEL_SPEED = 0x472,
//    BACK_RIGHT_WHEEL_SPEED = 0x473
} CAN_ID;

CAN_TxHeaderTypeDef TxHeader;
uint8_t MC_Command_MSG_Data[8];
uint32_t TxMailbox;
CAN_TxHeaderTypeDef TxHeader;
uint8_t VCU_STATE_MSG[8];
uint32_t TxMailbox;
uint32_t RxFifo[];
void can_init();
void can_receive();
void can_tx_vcu_state();
void can_tx_torque_request();
void can_tx_disable_MC();


#endif /* SRC_CAN_MANAGER_H_ */

