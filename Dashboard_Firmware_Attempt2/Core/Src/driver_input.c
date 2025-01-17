/*
 * driver_input.c
 *
 *  Created on: Jan 6, 2025
 *      Author: Melody
 */
#include <stdio.h>
#include "driver_input.h"
#include "stm32f7xx_hal.h"
#include "frucd_display.h"
#include "ugui.h"

#define NUM_BUTTONS 6

typedef struct {
	volatile uint32_t last_valid_pressed_time;
	volatile uint8_t enabled;
//	void (*onEnable)();
} button_state_t;

button_state_t button_states[NUM_BUTTONS] = {
	{last_valid_pressed_time: 0, enabled: 0}, // TC
	{last_valid_pressed_time: 0, enabled: 0}, // debug
	{last_valid_pressed_time: 0, enabled: 0}, // marker
	{last_valid_pressed_time: 0, enabled: 0}, // overtake
	{last_valid_pressed_time: 0, enabled: 0}, // HV
	{last_valid_pressed_time: 0, enabled: 0} // drive
};

button_id_t check_button_pressed() {
	if (!HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_12)) {
		return TC_BUTTON;
	}
	if (!HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_11)) {
		return DEBUG_BUTTON;
	}
	if (!HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10)) {
		return MARKER_BUTTON;
	}
	if (!HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)) {
		return OVERTAKE_BUTTON;
	}
	return NO_BUTTON;

}

uint8_t is_button_enabled(button_id_t button_id) {
	return button_states[button_id].enabled;
}

char disp_str[] = "   "; // 3 spaces

// runs once only when initially enabled
void on_button_enabled(button_id_t enabled_id) {
	switch (enabled_id) {
		case DEBUG_BUTTON:
			Display_DebugTemplate();
			break;
		case TC_BUTTON:
			sprintf(disp_str, "TC ");
			UG_PutString(5, 250, disp_str);
			break;
		case MARKER_BUTTON:
			sprintf(disp_str, "MRK");
			UG_PutString(100, 250, disp_str);
			break;
		case OVERTAKE_BUTTON:
			sprintf(disp_str, "OVT");
			UG_PutString(50, 250, disp_str);
			break;
		default:

	}
}

// runs once when disabled
void on_button_disabled(button_id_t disabled_id) {
	switch (disabled_id) {
		case DEBUG_BUTTON:
			Display_DriveTemplate();
			break;
		case TC_BUTTON:
			sprintf(disp_str, "   ");
			UG_PutString(5, 250, disp_str);
			break;
		case MARKER_BUTTON:
			sprintf(disp_str, "   ");
			UG_PutString(100, 250, disp_str);
			break;
		case OVERTAKE_BUTTON:
			sprintf(disp_str, "   ");
			UG_PutString(50, 250, disp_str);
			break;
		default:
	}
}

void driver_input_update() {
	button_id_t pressed_btn_id = check_button_pressed();
	if (pressed_btn_id != NO_BUTTON) {
		int32_t time_diff = HAL_GetTick() - button_states[pressed_btn_id].last_valid_pressed_time;
		if (time_diff < 0) {
			// timer overflowed and wrapped around
			time_diff = SYSTICK_MAX_VAL + time_diff;
		}

		// enable feature after button stays "pressed" long enough, avoids oscillating presses
		// less of an issue if electrical implemented hardware button debouncing :(
		if (time_diff > BUTTON_DELAY) {
			button_states[pressed_btn_id].last_valid_pressed_time = HAL_GetTick();

			if (button_states[pressed_btn_id].enabled) {
				button_states[pressed_btn_id].enabled = 0;
				on_button_disabled(pressed_btn_id);

			}
			else {
				button_states[pressed_btn_id].enabled = 1;
				on_button_enabled(pressed_btn_id);
			}
		}
	}
}

uint8_t hv_switch() {
	return !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13);
}

uint8_t drive_switch() {
	return !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14);
}

