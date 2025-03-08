/*
 * driver_practice.c
 *
 *  Created on: Jan 7, 2025
 *      Author: Vicle
 */

 #include "driver_practice.h"
 #include "xsens.h"
 #include "frucd_display.h"
 #include <stdint.h>
 #include <math.h>
 
 // Blue Max Kart Club's start coordinates are (very roughly) 38.575929, -121.732505
 // user should set start coordinates using initialize_start_coordinates()
 volatile float start_latitude = 38.575929;
 volatile float start_longitude = -121.732505;
 
 volatile float curr_latitude = 0.0;
 volatile float curr_longitude = 0.0;
 
 // Feet = 364,320 x Degrees
 const uint8_t tolerance_feet = 5;
 
 bool is_first_run = true;
 volatile bool is_recording_lap_time = false;
 volatile bool is_finishing_lap = false;
 
 volatile uint32_t prev_tick = 0;
 
 volatile uint32_t best_lap_time_ticks = 41 * 1000;
 volatile uint32_t curr_lap_time_ticks = 0;
 
 
 /* PRIVATE FUNCTIONS */
 
 void update_curr_coordinates() {
     curr_latitude = xsens_latitude;
     curr_longitude = xsens_longitude;
 }
 
 void update_curr_lap_time() {
     uint32_t ticks_passed = 0;
     uint32_t curr_tick = HAL_GetTick();
 
     if (curr_tick < prev_tick) { // HAL_GetTick overflowed
         ticks_passed = curr_tick + (SysTick->LOAD - prev_tick);
     } else {
         ticks_passed = curr_tick - prev_tick;
     }
 
     prev_tick = curr_tick;
     curr_lap_time_ticks += ticks_passed;
 }
 
 void update_is_recording_lap_time() {
     if (!is_recording_lap_time && is_first_run) {
         if (fabs(curr_latitude - start_latitude) > tolerance_feet / 364320.0 &&
                 fabs(curr_longitude - start_longitude) > tolerance_feet / 364320.0) {
             is_recording_lap_time = true;
         }
     }
 }
 
 void update_is_finishing_lap() {
     if (is_recording_lap_time) {
         if (fabs(curr_latitude - start_latitude) < tolerance_feet / 364320.0 &&
                 fabs(curr_longitude - start_longitude) < tolerance_feet / 364320.0) {
             is_finishing_lap = true;
         } else {
             is_finishing_lap = false;
         }
     }
 }
 
 /* PUBLIC FUNCTIONS */
 
 // Should be called once when driver starts practice mode
 void initialize_start_coordinates() {
     start_latitude = xsens_latitude;
     start_longitude = xsens_longitude;
 }
 
 // Should be called regularly after start coordinates have been initialized
 void driver_practice_update() {
     union Location_Data locData;
     locData.latLong[0] = xsens_latitude;
     locData.latLong[1] = xsens_longitude;
     // FIXME: temporary displaying
     //Display_Location(locData.latLong_ints[0], locData.latLong_ints[1]);
     if (!is_recording_lap_time) { // driver has not started their run
         update_is_recording_lap_time();
         return;
     }
     if (is_first_run) {
         is_first_run = false;
         prev_tick = HAL_GetTick();
     }
 
     update_curr_coordinates();
     update_curr_lap_time();
     update_is_finishing_lap();
 
     if (is_finishing_lap) { // currently crossing start line
         while (is_finishing_lap) {
             update_is_finishing_lap();
         }
 
         // just finished a lap
         float time_diff_sec = curr_lap_time_ticks / 1000.0 - best_lap_time_ticks / 1000.0;
 
 
 
 
 
         if (curr_lap_time_ticks < best_lap_time_ticks) {
             best_lap_time_ticks = curr_lap_time_ticks;
         }
 
         curr_lap_time_ticks = 0;
     }
 
 }
 
