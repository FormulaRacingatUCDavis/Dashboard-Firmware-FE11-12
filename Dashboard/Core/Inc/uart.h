/*
 * uart.h
 *
 *  Created on: Dec 2, 2023
 *      Author: sruti
 */

#ifndef INC_UART_H_
#define INC_UART_H_



#include "stdio.h"
#include "sensors.h"
#include "fsm.h"

#define ESCAPE_CHAR 0x05
#define FRAME_START 0x01
#define FRAME_END 0x0A

void print_state();
void print_pedal_vals();
void clear_screen();

void gui_dump();
void send_byte_with_escape(uint8_t byte);
void uart_write(uint8_t byte);




#endif /* INC_UART_H_ */
