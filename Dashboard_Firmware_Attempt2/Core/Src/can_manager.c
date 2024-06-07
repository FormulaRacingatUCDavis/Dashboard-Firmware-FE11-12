#include "can_manager.h"
#include "sd_card.h"
#include "serial_print.h"
#include "traction_control.h"

volatile uint8_t mc_lockout;
volatile uint8_t mc_enabled;
volatile uint16_t capacitor_volt = 0;
volatile uint8_t shutdown_flags = 0b0011111000;  //start with shutdown flags OK
volatile uint8_t estop_flags = 0;
volatile uint8_t switches = 0xC0;   //start with switches on to stay in startup state
volatile uint8_t PACK_TEMP;
volatile uint8_t mc_fault;
volatile uint8_t soc;
volatile uint16_t bms_status;
volatile uint8_t mc_fault_clear_success = 0;
volatile uint16_t pack_voltage;
volatile uint16_t motor_temp;
volatile uint16_t mc_temp;
volatile int16_t glv_v;

volatile int16_t motor_speed = 0;
volatile uint16_t rear_right_wheel_speed = 0;
volatile uint16_t rear_left_wheel_speed = 0;
volatile uint8_t wheel_updated[2] = {1,0};
volatile int16_t inlet_temp = 0;
volatile int16_t outlet_temp = 0;
volatile int16_t inlet_pres = 0;
volatile int16_t outlet_pres = 0;
volatile uint16_t telem_id = 0;
volatile uint16_t sg_rear = 0;

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

uint8_t mc_voltage_msg_counter = 0;
uint8_t mc_state_msg_counter = 0;
uint8_t mc_fault_msg_counter = 0;
uint8_t mc_motor_pos_msg_counter = 0;
uint8_t mc_glv_msg_counter = 0;
uint8_t mc_temp_msg_counter = 0;
uint8_t motor_temp_msg_counter = 0;
uint8_t torque_request_msg_counter = 0;
uint8_t vcu_state_msg_counter = 0;
uint8_t tc_sg_msg_counter = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	save_can_rx_data(RxHeader, RxData);
}


/************ CAN RX ************/

void save_can_rx_data(CAN_RxHeaderTypeDef RxHeader, uint8_t RxData[]) {
    // gets message and updates values
	switch (RxHeader.StdId) {
		case BMS_STATUS_MSG:
			PACK_TEMP = RxData[0];
			soc = RxData[1];
			bms_status = (RxData[2] << 8);
			bms_status += RxData[3];
			pack_voltage = (RxData[4] << 8);
			pack_voltage += RxData[5];

			write_rx_to_sd();
			break;
		case MC_VOLTAGE_INFO:
			capacitor_volt = (RxData[0] << 8); // upper bits
			capacitor_volt += RxData[1]; // lower bits

			if (mc_voltage_msg_counter == 0) write_rx_to_sd();
			mc_voltage_msg_counter++;
			mc_voltage_msg_counter %= 20;

			break;
		case MC_INTERNAL_STATES:
			mc_lockout = RxData[6] & 0b1000000;
			mc_enabled = RxData[6] & 0b1;

			if (mc_state_msg_counter == 0) write_rx_to_sd();
			mc_state_msg_counter++;
			mc_state_msg_counter %= 20;

			break;
		case PEI_CURRENT_SHUTDOWN:
			shutdown_flags = RxData[2];
			write_rx_to_sd();
			break;
		case MC_FAULT_CODES:
			if (mc_fault_msg_counter == 0) write_rx_to_sd();
			mc_fault_msg_counter++;
			mc_fault_msg_counter %= 20;

			for (uint8_t i = 0; i < 8; i++) {
				if (RxData[i] > 0) {
					mc_fault = 1;
					break;
				}
				else {
					mc_fault = 0;
				}
			}
			break;
		case MC_PARAM_RESPONSE:
			if (RxData[0] == 0x20 && RxData[2] == 1) {
				mc_fault_clear_success = 1;
			}
			break;
//		case WHEEL_SPEED_REAR:
//			rear_right_wheel_speed = (RxData[0] << 8);
//			rear_right_wheel_speed += RxData[1];
//			rear_left_wheel_speed = (RxData[2] << 8);
//			rear_left_wheel_speed += RxData[3];
//			wheel_updated[1] = 1;
//			telem_id = 0;
//			break;
		case MC_MOTOR_POSITION:
			motor_speed = (RxData[3] << 8);
			motor_speed |= RxData[2];
			motor_speed *= -1;

			// TEMPORARY?
			rear_right_wheel_speed = (RxData[3] << 8);
			rear_right_wheel_speed += RxData[2];
			rear_right_wheel_speed *= -1;
			wheel_updated[1] = 1;
			telem_id = 0;

			if (mc_motor_pos_msg_counter == 0) write_rx_to_sd();
			mc_motor_pos_msg_counter++;
			mc_motor_pos_msg_counter %= 5;

			break;
		case COOLING_LOOP:
			inlet_temp = (RxData[0] << 8);
			inlet_temp += RxData[1];
			outlet_temp = (RxData[2] << 8);
			outlet_temp += RxData[3];
			inlet_pres = (RxData[4] << 8);
			inlet_pres += RxData[5];
			outlet_pres = (RxData[6] << 8);
			outlet_pres += RxData[7];
			telem_id = 1;

			write_rx_to_sd();
			break;
		case MC_TEMP_3:
			motor_temp = RxData[5] << 8;
			motor_temp += RxData[4];

			if (motor_temp_msg_counter == 0) write_rx_to_sd();
			motor_temp_msg_counter++;
			motor_temp_msg_counter %= 20;

			break;
		case MC_TEMP_1:
			uint16_t module_a_temp = (RxData[1] << 8) + RxData[0];
			uint16_t module_b_temp = (RxData[3] << 8) + RxData[2];
			uint16_t module_c_temp = (RxData[5] << 8) + RxData[4];
			mc_temp = (module_a_temp + module_b_temp + module_c_temp) / 3; // no unit conversion, don't want to store float

			if (mc_temp_msg_counter == 0) write_rx_to_sd();
			mc_temp_msg_counter++;
			mc_temp_msg_counter %= 20;

			break;
		case MC_INTERNAL_VOLTS:
			glv_v = RxData[7] << 8;
			glv_v += RxData[6]; // no unit conversion, don't want to store float

			if (mc_glv_msg_counter == 0) write_rx_to_sd();
			mc_glv_msg_counter++;
			mc_glv_msg_counter %= 100;

			break;
		case STRAIN_GAUGE_REAR:
			sg_rear = RxData[0] << 8;
			sg_rear += RxData[1];
			break;
		default:
			// no valid input received
			break;
	}

}


/************ CAN TX ************/

CAN_TxHeaderTypeDef   TxHeader;
uint32_t              TxMailbox;

//  transmit state
void can_tx_vcu_state(CAN_HandleTypeDef *hcan){
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = VEHICLE_STATE;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	uint16_t tick = (uint16_t)HAL_GetTick();
	uint8_t data_tx_state[8] = {
        0,
        hv_requested(),
        throttle1.percent,
        throttle2.percent,
		brake.percent,
        one_byte_state(),
		(tick >> 8) & 0xFF,
		tick & 0xFF
    };

	if (vcu_state_msg_counter == 0) write_tx_to_sd(TxHeader, data_tx_state);
	vcu_state_msg_counter++;
	vcu_state_msg_counter %= 2;

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx_state, &TxMailbox) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
//    write_tx_to_sd(TxHeader, data_tx_state);
}

HAL_StatusTypeDef CAN_Send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t* data, uint8_t len)
{
	static CAN_TxHeaderTypeDef msg_hdr;
	msg_hdr.IDE = CAN_ID_STD;
	msg_hdr.StdId = id;
	msg_hdr.RTR = CAN_RTR_DATA;
	msg_hdr.DLC = len;

	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) return HAL_OK;
	return HAL_CAN_AddTxMessage(hcan, &msg_hdr, data, &TxMailbox);
}

//  transmit state
void can_tx_sg(CAN_HandleTypeDef *hcan, uint16_t adc){
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x500;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 6;
	uint8_t data_tx_state[6] = {
		(adc >> 8) & 0xFF,
		(adc & 0xFF),
		front_right_wheel_speed >> 8,
		front_right_wheel_speed & 0xff,
//		front_left_wheel_speed >> 8,
//		front_left_wheel_speed & 0xff,
		TC_torque_req  >> 8,
		TC_torque_req & 0xff,
    };

	if (tc_sg_msg_counter == 0) write_tx_to_sd(TxHeader, data_tx_state);
	tc_sg_msg_counter++;
	tc_sg_msg_counter %= 2;

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx_state, &TxMailbox) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
//    write_tx_to_sd(TxHeader, data_tx_state);
}


// transmit torque request
void can_tx_torque_request(CAN_HandleTypeDef *hcan){
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = TORQUE_REQUEST;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;

    uint8_t byte5 = 0b010;   //speed mode | discharge_enable | inverter enable
    uint16_t throttle_msg_byte = 0;
    if (state == DRIVE) {
    	byte5 |= 0x01;  //set inverter enable bit
    	throttle_msg_byte = requested_throttle();
    }

    uint8_t data_tx_torque[8] = {
        (uint8_t)(throttle_msg_byte & 0xff), // 0 - torque command lower (Nm*10)
        (uint8_t)(throttle_msg_byte >> 8) & 0xFF, // 1 - torque command upper (Nm*10)
        0, // 2 - speed command lower (not applicable)
        0, // 3 - speed command upper (not applicable)
        1, // 4 - direction (1 = forward, 0 = backward)
        byte5, // 5 - speed mode | discharge_enable | inverter enable
        0, // 6 - torque limit lower (if 0, default EEPROM value used)
        0 // 7 - torque limit upper (if 0, default EEPROM value used)
    };

    if (torque_request_msg_counter == 0) write_tx_to_sd(TxHeader, data_tx_torque);
	torque_request_msg_counter++;
	torque_request_msg_counter %= 2;

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx_torque, &TxMailbox) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
//    write_tx_to_sd(TxHeader, data_tx_torque);
}


void can_tx_disable_MC(CAN_HandleTypeDef *hcan) {
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = TORQUE_REQUEST;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;

	uint8_t data_tx_torque[8] = {0,0,0,0,0,0,0,0};

	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx_torque, &TxMailbox) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
//	write_tx_to_sd(TxHeader, data_tx_torque);
}

void can_clear_MC_fault(CAN_HandleTypeDef *hcan) {
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = MC_PARAM_COMMAND;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;

	const uint16_t param_addr = 20;
	uint8_t data_tx_param_command[8] = {
			param_addr & 0xFF, // address lower (little endian)
			param_addr >> 8, // address upper
			1, // r/w: 1 = write
			0, // reserved
			0, // data
			0, // data
			0, // reserved
			0 // reserved
	};

	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx_param_command, &TxMailbox) != HAL_OK)
	{
	  print("CAN Tx failed\r\n");
	}
//	write_tx_to_sd(TxHeader, data_tx_param_command);
}


