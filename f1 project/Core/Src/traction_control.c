/*
 * traction_control.c
 *
 *  Created on: Nov 11, 2023
 *      Author: cogus
 */

#include "traction_control.h"

extern volatile state_t state;
extern volatile uint16_t front_right_wheel_speed;
extern volatile uint16_t front_left_wheel_speed;
extern volatile uint16_t back_right_wheel_speed;
extern volatile uint16_t back_left_wheel_speed;

void traction_control_PID() {
    if (state != DRIVE) return;

    // note: wheel speeds are in units of pulses/20ms
    const float avg_front_wheel_speed = (front_right_wheel_speed + front_left_wheel_speed)/2.0;
    const float avg_back_wheel_speed = (back_right_wheel_speed + back_left_wheel_speed)/2.0;
    const float conversion_factor = (2*pi*wheel_radius)/pulses_per_rev;
    const float current_slip_ratio = (avg_back_wheel_speed*conversion_factor) / (avg_front_wheel_speed*conversion_factor);

    // if target slip ratio has been achieved
//    if (current_slip_ratio < target_slip_ratio + 0.001 || current_slip_ratio > target_slip_ratio - 0.001) return;

    pid_error = target_slip_ratio - current_slip_ratio;
    integral = integral + pid_error;
    derivative = pid_error - prev_pid_error;

    TC_control_var = (kP * pid_error) + (kI * integral) + (kD * derivative);

    // limit PID torque request
    if (TC_control_var > TC_torque_limit) TC_control_var = TC_torque_limit;
    if (TC_control_var < 0) TC_control_var = 0; // not the best way but works for now

    TC_torque_adjustment = TC_control_var;

    prev_pid_error = pid_error;
}

