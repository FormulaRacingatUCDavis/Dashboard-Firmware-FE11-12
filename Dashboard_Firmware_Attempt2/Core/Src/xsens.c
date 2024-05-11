/*
 * xsens.c
 *
 *  Created on: Mar 4, 2024
 *      Author: leoja
 */


#include "serial.h"
#include "xsens/xsens_mti.h"      // Main xsens library
#include "xsens/xsens_utility.h"  // Needed for quaternion conversion function
#include "fatfs.h"

#include <stdbool.h>

#define BUFLEN 200

// PRIVATE FUNCTION PROTOTYPES
void imu_callback(XsensEventFlag_t event, XsensEventData_t *mtdata);


// PUBLIC FUNCTION DEFINITIONS

void Xsens_Update(UART_HandleTypeDef* h_uart){
	static bool first_run = true;
	static Serial_t serial;
	static uint8_t rx_buf[BUFLEN];
	static xsens_interface_t imu_interface = XSENS_INTERFACE_RX(&imu_callback);


	if(first_run){
		Serial_Init(&serial, h_uart, rx_buf, BUFLEN);
		Serial_StartListening(&serial);
		first_run = false;
	}

	uint32_t b = Serial_BytesAvailable(&serial);
	for(uint32_t i = 0; i < b; i++){
	 xsens_mti_parse(&imu_interface, Serial_GetByte(&serial));
	}
}



// PRIVATE FUNCTION DEFINITIONS


// Called when the library decoded an inbound packet
//   - If the packet was an MData2 frame (which contains packed motion data)
//   - the callback is called once for each sub-field
void imu_callback(XsensEventFlag_t event, XsensEventData_t *mtdata)
{
    // The library provides a pointer to the a union containing decoded data
    // Use XsensEventFlag_t to determine what kind of packet arrived,
    // then copy data from the union as needed.

    // union
    // {
    //     uint8_t u1;
    //     uint16_t u2;
    //     uint32_t u4;
    //     float    f4;
    //     float    f4x2[2];
    //     float    f4x3[3];
    //     float    f4x4[4];
    //     float    f4x9[9];
    // } data;

	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

    switch( event )
    {
        case XSENS_EVT_DELTA_V:
          if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
            {
                //writeLine(0x999, 3, mtdata->data.f4x3);
            }
            break;

        case XSENS_EVT_EULER:
          if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
            {
        	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
                //writeLine(0xAAA, 3, mtdata->data.f4x3);
            }
            break;

        case XSENS_EVT_FREE_ACCELERATION:
          if(mtdata->type == XSENS_EVT_TYPE_FLOAT3)
            {
                //writeLine(0xBBB, 3, mtdata->data.f4x3);
            }
            break;

        case XSENS_EVT_LAT_LON:
        	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
        	break;

        default:
        	// IGNORE
        	break;
    }
}
