/*
 * driver_practice.h
 *
 *  Created on: Jan 7, 2025
 *      Author: Vicle
 */

 #ifndef INC_DRIVER_PRACTICE_H_
 #define INC_DRIVER_PRACTICE_H_
 
 #include <stdbool.h>
 #include <stdint.h>
 
 extern volatile float start_latitude;
 extern volatile float start_longitude;
 extern volatile float curr_latitude;
 extern volatile float curr_longitude;
 extern bool is_first_run;
 extern volatile bool is_recording_laptime;
 extern volatile bool is_finishing_lap;
 
 extern volatile uint32_t curr_lap_start_tick;
 extern volatile uint32_t best_lap_time_ticks;
 extern volatile uint32_t curr_lap_time_ticks;
 
 /* PRIVATE FUNCTIONS */
 void update_curr_coordinates();
 void update_curr_laptime();
 void update_is_recording_laptime();
 void update_is_finishing_lap();
 
 /* PUBLIC FUNCTIONS */
 void driver_practice_update();
 void initialize_start_coordinates();
 
 
 #endif /* INC_DRIVER_PRACTICE_H_ */
 