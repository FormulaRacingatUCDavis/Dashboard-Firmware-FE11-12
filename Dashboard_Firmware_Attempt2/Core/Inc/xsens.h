/*
 * xsens.h
 *
 *  Created on: Mar 4, 2024
 *      Author: leoja
 */

 #ifndef INC_XSENS_H_
 #define INC_XSENS_H_
 
 #include "stm32f7xx_hal.h"
 
 //PUBLIC FUNCTION PROTOTYPES
 void Xsens_Update(UART_HandleTypeDef* h_uart);
 
 void log_xsens(UART_HandleTypeDef* xsens_uart, UART_HandleTypeDef* log_uart);
 
 // Public MTData2 variables
 extern float xsens_latitude;
 extern float xsens_longitude;
 
 union Location_Data {
	 float latLong[2];
	 uint8_t latLong_ints[8];
 };
 
 #endif /* INC_XSENS_H_ */
 