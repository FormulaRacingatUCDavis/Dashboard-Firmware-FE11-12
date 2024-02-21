/* ========================================
 *
 * APIs for updating FRUCD Dashboard display
 *
 * ========================================
*/
#include <project.h>
#include <stdio.h>
#include "data.h"
#include "ugui.h"
#include "ugui_SSD1963.h"

// color caching variables
//  - avoid redrawing rectangle if color didn't change
UG_COLOR last_SOC_color;
UG_COLOR last_max_pack_temp_color;
UG_COLOR last_state_color;
UG_COLOR last_glv_v_color;
UG_COLOR last_mc_temp_color;
UG_COLOR last_motor_temp_color;
UG_COLOR last_shutdown_color;

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
