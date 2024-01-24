/*
 * frucd_display.h
 *
 *  Created on: Dec 4, 2023
 *      Author: leoja
 */

#ifndef INC_FRUCD_DISPLAY_H_
#define INC_FRUCD_DISPLAY_H_

/* ========================================
 *
 * APIs for updating FRUCD Dashboard display
 *
 * ========================================
*/

#include <stdio.h>
#include "data.h"
#include "ugui.h"
#include "ugui_SSD1963.h"

// Just a test function that displays elements at the supposed corners of the screen
void calibrateScreen();

// Initializes dashboard layout/labels
//void initDashTemplate();

// Initialize debug mode or drive mode
void debugTemplate();
void driveTemplate();
void clear_colors(void);


/*
 * BEGIN: functions to show data on dash
 * - color decision logic is in here
 */
void disp_SOC(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
void disp_max_pack_temp(uint8_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
void disp_state(uint8_t state, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
void disp_glv_v(uint32_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
void disp_mc_temp(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
void disp_motor_temp(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
void disp_gen_temp(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
void disp_gen_voltage(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
void disp_shutdown_circuit(uint8_t shutdown_flags, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
void disp_debug(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
void disp_mc_fault(uint8_t data[8], uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size);
/* [] END OF FILE */


#endif /* INC_FRUCD_DISPLAY_H_ */
