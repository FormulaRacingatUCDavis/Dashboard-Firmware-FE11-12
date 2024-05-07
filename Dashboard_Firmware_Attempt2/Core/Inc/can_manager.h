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
    VEHICLE_STATE = 0x766,
    TORQUE_REQUEST = 0x0C0,
    BMS_STATUS_MSG = 0x380,
    PEI_CURRENT_SHUTDOWN = 0x387,
    MC_VOLTAGE_INFO = 0x0A7,
    MC_INTERNAL_STATES = 0xAA,
    MC_FAULT_CODES = 0xAB,
	MC_PARAM_COMMAND = 0x0C1,
	MC_PARAM_RESPONSE = 0x0C2,
	MC_MOTOR_POSITION = 0x0A5,
    WHEEL_SPEED_REAR = 0x401
} CAN_ID;

extern volatile uint8_t mc_lockout;
extern volatile uint8_t mc_enabled;
extern volatile uint16_t capacitor_volt;
extern volatile uint8_t shutdown_flags;
extern volatile uint8_t estop_flags;
extern volatile uint8_t switches;
extern volatile uint8_t PACK_TEMP;
extern volatile uint8_t mc_fault;
extern volatile uint8_t soc;
extern volatile uint16_t bms_status;
extern volatile uint8_t mc_fault_clear_success;
extern volatile uint16_t pack_voltage;

extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];

extern CAN_TxHeaderTypeDef   TxHeader;
extern uint32_t              TxMailbox;

// From TCAN
extern volatile uint16_t back_right_wheel_speed;
extern volatile uint16_t back_left_wheel_speed;

void save_can_rx_data(CAN_RxHeaderTypeDef RxHeader, uint8_t RxData[]);
void can_tx_vcu_state(CAN_HandleTypeDef *hcan);
void can_tx_torque_request(CAN_HandleTypeDef *hcan);
void can_tx_disable_MC(CAN_HandleTypeDef *hcan);
void can_clear_MC_fault(CAN_HandleTypeDef *hcan);

#endif /* SRC_CAN_MANAGER_H_ */

