/*
 * sensors.c
 *
 *  Created on: Feb 20, 2024
 *      Author: cogus
 */
#include "sensors.h"
#include "can_manager.h"
#include "frucd_display.h"
#include "driver_input.h"
#include <stdlib.h>
#include "traction_control.h"

CALIBRATED_SENSOR_t throttle1;
CALIBRATED_SENSOR_t throttle2;
CALIBRATED_SENSOR_t brake;
volatile uint32_t torque_percentage = 0;
volatile uint32_t launch_control_param = 0;
volatile uint32_t prev_torque_percentage = 0;
volatile uint32_t prev_launch_control_param = 0;
volatile uint32_t torque_req = 0;

#define RADS_PER_RPM 0.10472
#define MAX_TORQUE_OVERTAKE (uint16_t)(MAX_TORQUE_NM * 0.8)

extern volatile uint8_t traction_control_enabled;
extern volatile int16_t motor_speed;
extern volatile uint16_t acc_current_adc;
extern volatile uint16_t acc_current_ref_adc;
extern volatile int16_t pack_voltage;
extern volatile uint8_t soc;

uint16_t get_max_torque(uint32_t max_power);
uint32_t get_max_power();
//extern void Error_Handler();


// RPM to Torque Table for Power Limiter 2.0
// Usage: rpm_to_torque_table[rpm - MIN_SPEED_FOR_SMOOTHING] => torque at that rpm, where MIN_SPEED_FOR_SMOOTHING is 3474
// rpm starts affecting torque limit at 3474 rpm, for lower rpms can simply use 220 Nm (default max)
// max rpm that can be used for indexing is 6049 rpm (macro MAX_SPEED_FOR_SMOOTHING)
uint8_t rpm_to_torque_table[] = {219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,218,218,218,218,218,218,218,218,218,218,218,218,218,218,218,218,217,217,217,217,217,217,217,217,217,217,217,217,217,217,217,217,216,216,216,216,216,216,216,216,216,216,216,216,216,216,216,216,215,215,215,215,215,215,215,215,215,215,215,215,215,215,215,215,215,214,214,214,214,214,214,214,214,214,214,214,214,214,214,214,214,213,213,213,213,213,213,213,213,213,213,213,213,213,213,213,213,213,212,212,212,212,212,212,212,212,212,212,212,212,212,212,212,212,212,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,211,210,210,210,210,210,210,210,210,210,210,210,210,210,210,210,210,210,209,209,209,209,209,209,209,209,209,209,209,209,209,209,209,209,209,209,208,208,208,208,208,208,208,208,208,208,208,208,208,208,208,208,208,207,207,207,207,207,207,207,207,207,207,207,207,207,207,207,207,207,207,206,206,206,206,206,206,206,206,206,206,206,206,206,206,206,206,206,206,205,205,205,205,205,205,205,205,205,205,205,205,205,205,205,205,205,205,204,204,204,204,204,204,204,204,204,204,204,204,204,204,204,204,204,204,203,203,203,203,203,203,203,203,203,203,203,203,203,203,203,203,203,203,203,202,202,202,202,202,202,202,202,202,202,202,202,202,202,202,202,202,202,201,201,201,201,201,201,201,201,201,201,201,201,201,201,201,201,201,201,201,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,199,199,199,199,199,199,199,199,199,199,199,199,199,199,199,199,199,199,199,199,198,198,198,198,198,198,198,198,198,198,198,198,198,198,198,198,198,198,198,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,197,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,195,195,195,195,195,195,195,195,195,195,195,195,195,195,195,195,195,195,195,195,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,194,193,193,193,193,193,193,193,193,193,193,193,193,193,193,193,193,193,193,193,193,193,192,192,192,192,192,192,192,192,192,192,192,192,192,192,192,192,192,192,192,192,191,191,191,191,191,191,191,191,191,191,191,191,191,191,191,191,191,191,191,191,191,190,190,190,190,190,190,190,190,190,190,190,190,190,190,190,190,190,190,190,190,190,189,189,189,189,189,189,189,189,189,189,189,189,189,189,189,189,189,189,189,189,189,189,188,188,188,188,188,188,188,188,188,188,188,188,188,188,188,188,188,188,188,188,188,187,187,187,187,187,187,187,187,187,187,187,187,187,187,187,187,187,187,187,187,187,187,186,186,186,186,186,186,186,186,186,186,186,186,186,186,186,186,186,186,186,186,186,186,185,185,185,185,185,185,185,185,185,185,185,185,185,185,185,185,185,185,185,185,185,185,184,184,184,184,184,184,184,184,184,184,184,184,184,184,184,184,184,184,184,184,184,184,183,183,183,183,183,183,183,183,183,183,183,183,183,183,183,183,183,183,183,183,183,183,183,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,182,181,181,181,181,181,181,181,181,181,181,181,181,181,181,181,181,181,181,181,181,181,181,181,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,180,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,179,178,178,178,178,178,178,178,178,178,178,178,178,178,178,178,178,178,178,178,178,178,178,178,178,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,177,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,174,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,173,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,172,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,171,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,169,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,168,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,167,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,166,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,164,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,163,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,162,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,161,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,159,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,158,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,157,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,156,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,155,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,153,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,152,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,151,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,149,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,148,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,147,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,146,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,145,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,143,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,141,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,139,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,138,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,137,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,135,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,133,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,132,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,129,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126,126};


void init_sensors(){
    throttle1.min = 0x7FFF;
    throttle1.max = 0;
    throttle1.range = 1;
    throttle2.min = 0x7FFF;
    throttle2.max = 0;
    throttle2.range = 1;
    brake.min = 0x7FFF;
    brake.max = 0;
    brake.range = 1;
}

void select_adc_channel(ADC_HandleTypeDef *hadc, ADC_CHAN channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    switch (channel)
    {
        case APPS1:
            sConfig.Channel = ADC_CHANNEL_10;
			sConfig.Rank = 1;

			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;

        case APPS2:
			sConfig.Channel = ADC_CHANNEL_8;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case BSE:
			sConfig.Channel = ADC_CHANNEL_15;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case KNOB1:
			sConfig.Channel = ADC_CHANNEL_13;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case KNOB2:
			sConfig.Channel = ADC_CHANNEL_12;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        case STRAIN_GAUGE:
			sConfig.Channel = ADC_CHANNEL_11;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
//				Error_Handler();
			}
			break;
        default:
            break;
    }
}

uint32_t get_adc_conversion(ADC_HandleTypeDef *hadc, ADC_CHAN channel) {

	select_adc_channel(hadc, channel);

	uint32_t conversion;

	HAL_ADC_Start(hadc);

	// Wait for the conversion to complete
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);

	// Get the ADC value
	conversion = HAL_ADC_GetValue(hadc);

	return conversion;
}


// Update sensors

void run_calibration() {
    update_minmax(&throttle1);
    update_minmax(&throttle2);
    update_minmax(&brake);
}

void update_sensor_vals(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc3) {
	// pedals
    throttle1.raw = get_adc_conversion(hadc1, APPS1);
    update_percent(&throttle1);
    throttle2.raw = get_adc_conversion(hadc3, APPS2);
    update_percent(&throttle2);
    brake.raw = get_adc_conversion(hadc3, BSE);
    update_percent(&brake);

    // knobs
    prev_torque_percentage = torque_percentage;
	prev_launch_control_param = launch_control_param;
    torque_percentage = get_adc_conversion(hadc1, KNOB2) * 100 / 4095;
    launch_control_param = get_adc_conversion(hadc1, KNOB1) * 100 / 4095;
}

static float raw_to_mvolts(uint16_t adc_raw) {
    return ((float)adc_raw / 4095) * 3.3 * 1000;
}

static float mvolts_to_amps(float mVolts, float mVolt_ref) {
    return ((mVolts - mVolt_ref) * 7.4 / 4.7) / 6.667;
}

static float get_accumulator_power() {
	float acc_current_amps = mvolts_to_amps(raw_to_mvolts(acc_current_adc), raw_to_mvolts(acc_current_ref_adc));
	float acc_voltage_volt = pack_voltage * 0.018 + 180;

	return acc_current_amps*acc_voltage_volt;
}

int16_t requested_throttle(){
    uint32_t max_power = get_max_power();
    uint16_t max_torque = get_max_torque(max_power);

    // zero throttle if brake is pressed at all, prevents hardware bspd
//	if (brake.percent >= BRAKE_BSPD_THRESHOLD) return 0;


    // MAKE EXTRA SURE 80kW accumulator power draw is not exceeded or FUSE WILL BLOW
	if (get_accumulator_power() >= MAX_POWER_ACCUMULATOR_W) {// if exceed power limit of 70kW, severely limit torque
		if (max_torque > MAX_TORQUE_AT_POWER_LIMIT) {
			// torque of 120 Nm is safe in theory even at 80kW and 6049 RPM
			// why max torque is not set to 0: would lead to lurching as power oscillates above and below 70kW
			max_torque = MAX_TORQUE_AT_POWER_LIMIT;
		}
	} else if (get_accumulator_power() >= SMOOTHING_POWER_THRESHOLD_W) { // start smoothing torque if getting close to 70kW
		if (motor_speed < MIN_RPM_FOR_SMOOTHING) {
			// don't need to limit torque if RPM is low enough
		} else if (motor_speed < MAX_RPM_FOR_SMOOTHING) { // use table to limit torque
			uint8_t max_torque_rpm = rpm_to_torque_table[motor_speed - MIN_RPM_FOR_SMOOTHING];
			if (max_torque > max_torque_rpm) {
				max_torque = max_torque_rpm;
			}
		} else { // if rpm is above 6049, can't use table, so just limit as if we are over 70kW
			if (max_torque > MAX_TORQUE_AT_POWER_LIMIT) {
				max_torque = MAX_TORQUE_AT_POWER_LIMIT;
			}
		}

	}

    torque_req = (throttle1.percent * max_torque * 10) / 100;  //upscale for MC code, Nm times 10

    // use reduced values from TC if TC torque request is lower
    if(is_button_enabled(TC_BUTTON) && (torque_req > TC_torque_req)){
		torque_req = TC_torque_req;
	}

    // regenerative braking:
    // EV.3.3.3 The powertrain must not regenerate energy when vehicle speed is between 0 and 5 km/hr
    // 0.016349 comes from (60 * pi * tire_diameter) / (FDR * 63360) where 63360 is conversion factor
    // 1.60934 is converting mph to kph
    float car_speed_kph = abs(motor_speed) * 0.016349 * 1.60934;
    // State of charge should also be below 95%, prevent overcharging accumulator
    if (throttle1.percent < DEADZONE_PERCENTAGE && car_speed_kph > 5 && soc < 95) {
    	// negative torque request for regen braking
    	float current_term = 40.5; // 40.5 amps
    	float acc_voltage_volt = pack_voltage * 0.018 + 180;
    	float voltage_term = abs(acc_voltage_volt);
    	float rpm_term = abs(motor_speed);

    	// TODO TEMPORARY: uses launch control knob to change intrusiveness of regen braking
    	int16_t regen_torque = (int16_t)( (launch_control_param / 100.0) * -1*voltage_term * current_term / (0.10472 * rpm_term) );

    	return clamp(regen_torque*10, -90*10, 0); // max 90 Nm on regen
    }


    return (uint16_t)torque_req;
}

// get maximum power based on power limit
// attenuate for BMS temps between 50 and 60
uint32_t get_max_power(){
	if(PACK_TEMP < 50) {
		return MAX_POWER_MOTOR_W;
	} else if(PACK_TEMP < 58) {
		return (58 - PACK_TEMP)*(MAX_POWER_MOTOR_W / 8);
	} else {
		return 0;
	}
}

uint16_t get_max_torque(uint32_t max_power){
	float motor_speed_rads = (float)motor_speed * RADS_PER_RPM;
	float max_torque_power = max_power / motor_speed_rads; // max torque calculated from max power
	float max_torque_knob = MAX_TORQUE_NM * (float)torque_percentage / 100;

	// if overtake is enabled, return the lower of torque limit set by overtake value and power limit
	if (is_button_enabled(OVERTAKE_BUTTON) && MAX_TORQUE_OVERTAKE < max_torque_power) {
		return MAX_TORQUE_OVERTAKE;
	}

	// return the lower of the torque limit set by the knob and by the power limit
	if (max_torque_knob < max_torque_power) {
		return (uint16_t)max_torque_knob;
	}
	else {
		return (uint16_t)max_torque_power;
	}
}

bool sensors_calibrated(){
	return throttle1.range > APPS1_MIN_RANGE &&
		   throttle2.range > APPS2_MIN_RANGE &&
		   brake.range > BRAKE_MIN_RANGE;
}

bool braking(){
    return brake.raw > BRAKE_LIGHT_THRESHOLD;
}

bool brake_mashed(){
    return brake.percent > RTD_BRAKE_THRESHOLD;
}

// check differential between the throttle sensors
// returns true only if the sensor discrepancy is > 10%
// Note: after verifying there's no discrepancy, can use either sensor(1 or 2) for remaining checks
bool has_discrepancy() {
    if(abs((int)throttle1.percent - (int)throttle2.percent) > 10) return 1;  //percentage discrepancy

    return (throttle1.raw < APPS_OPEN_THRESH)
        || (throttle1.raw > APPS_SHORT_THRESH)
        || (throttle2.raw < APPS_OPEN_THRESH)
        || (throttle2.raw > APPS_SHORT_THRESH);   // checks for wiring fault (open or short circuit).
    											  // not sure why this is in the discrepancy check function but it's needed
	return false;

}

// check for soft BSPD (in rules, called "APPS / Brake Pedal Plausibility Check")
// see EV.4.7 of FSAE 2025 rulebook
bool is_brake_implausible() {
    if (error == BRAKE_IMPLAUSIBLE) {
        // once brake implausibility detected,
        // can only revert to normal if throttle "unapplied"
        return !(throttle1.percent <= APPS1_BSPD_RESET_THRESHOLD);
    }

    // if brake applied and throttle > 25%, brake implausible
    return (brake.percent >= BRAKE_BSPD_THRESHOLD && throttle1.percent > APPS1_BSPD_THRESHOLD);
}

void update_percent(CALIBRATED_SENSOR_t* sensor){
    uint32_t raw = (uint32_t)clamp(sensor->raw, sensor->min, sensor->max);
    sensor->percent = (uint16_t)((100*(raw-sensor->min))/((sensor->range)));
}

void update_minmax(CALIBRATED_SENSOR_t* sensor){
    if (sensor->raw > sensor->max) sensor->max = sensor->raw;
    else if (sensor->raw < sensor->min) sensor->min = sensor->raw;
    if(sensor->max > sensor->min) sensor->range = sensor->max - sensor->min;
}

void add_apps_deadzone(){
	add_deadzone(&throttle1, 5);
	add_deadzone(&throttle2, 5);
	add_deadzone(&brake, 10);
}

void add_deadzone(CALIBRATED_SENSOR_t* sensor, uint16_t deadzone_percentage){
	uint16_t deadzone = sensor->range * deadzone_percentage / 100;

	// catch funky cases that would end up with a negative or 0 range
	if(deadzone >= sensor->range) return;

	sensor->min += deadzone;
	sensor->range -= deadzone;
}

int16_t clamp(int16_t in, int16_t min, int16_t max){
    if(in > max) return max;
    if(in < min) return min;
    return in;
}


