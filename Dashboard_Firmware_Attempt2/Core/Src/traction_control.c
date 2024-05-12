#include "traction_control.h"
#include "frucd_display.h"
#include "can_manager.h"
#include "stdio.h"

volatile uint16_t TC_control_var = 0;
volatile uint16_t TC_torque_adjustment = 0;

volatile uint16_t pid_error = 0;
volatile uint16_t prev_pid_error = 0;

volatile uint16_t integral = 0;
const uint16_t integral_cap = 0; // PLACEHOLDER
volatile uint16_t derivative = 0;

const float pi = 3.14;
const float wheel_radius = 0.5; // PLACEHOLDER VALUE
const uint16_t pulses_per_rev = 60; // from wheel speed sensor

float target_slip_ratio = 0.1;
volatile float current_slip_ratio = 0;

const float kP = 16;
const float kI = 1.6;
const float kD = 0; // probably don't need this term

const uint16_t TC_torque_limit = 100;

void traction_control_PID(uint32_t fr_wheel_speed, uint32_t fl_wheel_speed) {
    if (state != DRIVE) return;

    // units are in RPM/CPS
    const float avg_front_wheel_speed = (fr_wheel_speed + fl_wheel_speed)/2.0;
    const float avg_rear_wheel_speed = rear_right_wheel_speed*12.0/33; // (back_right_wheel_speed + back_left_wheel_speed)/2.0;
    current_slip_ratio = avg_rear_wheel_speed/avg_front_wheel_speed;

//    // calculate dynamic slip ratio
//    target_slip_ratio = 0.1 - 0.01*(avg_back_wheel_speed/max_wheel_speed);

    // if target slip ratio has been achieved
    if (current_slip_ratio < target_slip_ratio + 0.001 || current_slip_ratio > target_slip_ratio - 0.001) {
    	integral = 0; // reset integral
    	return;
    }

//    // if integral exceeds cap
//    if (integral > integral_cap) {
//    	integral = integral_cap;
//    }

    pid_error = target_slip_ratio - current_slip_ratio;
    integral = integral + pid_error;
    derivative = pid_error - prev_pid_error;

    TC_control_var = (kP * pid_error) + (kI * integral) + (kD * derivative);

    // limit PID torque request
    if (TC_control_var > TC_torque_limit) TC_control_var = TC_torque_limit;
    if (TC_control_var < 0) TC_control_var = 0;

    TC_torque_adjustment = TC_control_var; // this will be subtracted from the actual torque request

    prev_pid_error = pid_error;
}
