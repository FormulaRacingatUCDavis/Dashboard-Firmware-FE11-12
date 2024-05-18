/*
 * driver_input.h
 *
 *  Created on: May 17, 2024
 *      Author: Melody
 */

#ifndef INC_DRIVER_INPUT_H_
#define INC_DRIVER_INPUT_H_

#define BUTTON_DELAY 800

// traction control
volatile uint8_t traction_control_enabled = 0;
volatile uint32_t tc_last_valid_pressed_time = 0;

uint8_t traction_control_btn_pressed() {
	return !HAL_GPIO_ReadPin(GPIOG, BUTTON_1_Pin);
}

char disp_str[3];
void traction_control_enabled_update() {
	if (traction_control_btn_pressed()) {
		int32_t time_diff = HAL_GetTick() - tc_last_valid_pressed_time;
		if (time_diff < 0) time_diff = -time_diff; // absolute val time diff

		if (time_diff > BUTTON_DELAY) {
			tc_last_valid_pressed_time = HAL_GetTick();

			if (traction_control_enabled) {
				traction_control_enabled = 0;
				sprintf(disp_str, "  ");
				UG_PutString(415, 250, disp_str);

			}
			else {
				traction_control_enabled = 1;
				sprintf(disp_str, "TC");
				UG_PutString(415, 250, disp_str);
			}
		}
	}
}

// display debug mode
volatile uint8_t display_debug_enabled = 0;
volatile uint32_t debug_last_valid_pressed_time;

uint8_t debug_btn_pressed() {
	return !HAL_GPIO_ReadPin(GPIOG, BUTTON_2_Pin);
}

void debug_enabled_update() {
	if (debug_btn_pressed()) {
		int32_t time_diff = HAL_GetTick() - debug_last_valid_pressed_time;
		if (time_diff < 0) time_diff = -time_diff; // absolute val time diff

		if (time_diff > BUTTON_DELAY) {
			debug_last_valid_pressed_time = HAL_GetTick();

			if (display_debug_enabled) {
				display_debug_enabled = 0;
				Display_DriveTemplate();
			}
			else {
				display_debug_enabled = 1;
				Display_DebugTemplate();
			}
		}
	}
}

/************ Switches ************/

uint8_t hv_switch() {
	return !HAL_GPIO_ReadPin(GPIOG, HV_REQUEST_Pin);
}

uint8_t drive_switch() {
	return !HAL_GPIO_ReadPin(GPIOG, DRIVE_REQUEST_Pin);
}


#endif /* INC_DRIVER_INPUT_H_ */
