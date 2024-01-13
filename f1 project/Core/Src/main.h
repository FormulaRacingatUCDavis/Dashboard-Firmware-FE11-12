/*
 * main.h
 *
 *  Created on: Nov 11, 2023
 *      Author: cogus
 */

// This is a guard condition so that contents of this file are not included
// more than once.
#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_




#ifndef _XTAL_FREQ
#define _XTAL_FREQ  8000000UL
#endif

#ifndef FCY
#define FCY 8000000UL
#endif


// general includes
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdio.h>

// project includes
#include "can_manager.h"
#include "fsm.h"
#include "config.h"
#include "sensors.h"
#include "uart.h"
#include "traction_control.h"


int main();
void tmr1_ISR();
uint8_t hv_switch();
uint8_t drive_switch();
uint8_t shutdown_closed();
uint8_t traction_control_enable();





#endif /* SRC_MAIN_H_ */
