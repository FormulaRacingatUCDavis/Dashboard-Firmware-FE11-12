/*
 * uart.h
 *
 *  Created on: Feb 20, 2024
 *      Author: cogus
 */
#ifndef URT_H
#define URT_H

extern volatile state_t state;
extern volatile error_t error;
extern CALIBRATED_SENSOR_t throttle1;
extern CALIBRATED_SENSOR_t throttle2;
extern CALIBRATED_SENSOR_t brake;
extern volatile uint16_t capacitor_volt;

#endif


