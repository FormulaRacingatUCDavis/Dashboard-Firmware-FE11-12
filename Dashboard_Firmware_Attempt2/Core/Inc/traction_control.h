/*
 * traction_control.h
 *
 *  Created on: Feb 20, 2024
 *      Author: cogus
 */

#include <stdint.h>
#include "fsm.h"
#include "wheel_speed.h"

#ifndef SRC_TRACTION_CONTROL_H_
#define SRC_TRACTION_CONTROL_H_

extern volatile uint16_t TC_control_var;
extern volatile uint16_t TC_torque_adjustment;
extern volatile float current_slip_ratio;

extern volatile uint16_t pid_error;
extern volatile uint16_t prev_pid_error;

extern volatile uint16_t integral;
extern volatile uint16_t derivative;

void traction_control_PID(uint32_t fr_wheel_speed, uint32_t fl_wheel_speed);



#endif /* SRC_TRACTION_CONTROL_H_ */
