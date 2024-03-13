/*
 * can_manager.h
 *
 *  Created on: Feb 20, 2024
 *      Author: cogus
 */
#include <stdint.h>

#ifndef SRC_CAN_MANAGER_H_
#define SRC_CAN_MANAGER_H_

#include "stm32f7xx_hal.h"
#include "fsm.h"
#include "sensors.h"


/********** ENUM OF CAN IDS **********/
typedef enum {
    // PCAN
    //BSPD_FLAGS = 0x0c1,
    VEHICLE_STATE = 0x766,
    TORQUE_REQUEST = 0x0C0,
    BMS_STATUS_MSG = 0x380,
    PEI_CURRENT_SHUTDOWN = 0x387,
    MC_VOLTAGE_INFO = 0x0A7,
    MC_INTERNAL_STATES = 0xAA,
    MC_FAULT_CODES = 0xAB,
    // TCAN
    FRONT_LEFT_WHEEL_SPEED = 0x470,
    FRONT_RIGHT_WHEEL_SPEED = 0x471,
    BACK_LEFT_WHEEL_SPEED = 0x472,
    BACK_RIGHT_WHEEL_SPEED = 0x473
} CAN_ID;

extern volatile uint8_t mc_lockout;
extern volatile uint8_t mc_enabled;
extern volatile uint16_t capacitor_volt;
extern volatile uint8_t shutdown_flags;
extern volatile uint8_t estop_flags;
extern volatile uint8_t switches;
extern volatile uint8_t PACK_TEMP;
extern volatile uint8_t mc_fault;

// From TCAN
extern volatile uint16_t front_right_wheel_speed;
extern volatile uint16_t front_left_wheel_speed;
extern volatile uint16_t back_right_wheel_speed;
extern volatile uint16_t back_left_wheel_speed;

void save_can_rx_data(CAN_RxHeaderTypeDef RxHeader, uint8_t RxData[]);
void can_tx_vcu_state(CAN_HandleTypeDef *hcan);
void can_tx_torque_request(CAN_HandleTypeDef *hcan);
void can_tx_disable_MC(CAN_HandleTypeDef *hcan);


#endif /* SRC_CAN_MANAGER_H_ */

