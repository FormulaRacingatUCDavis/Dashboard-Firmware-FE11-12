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

// PUBLIC FUNCTION PROTOTYPES //
void Display_Init();
void Display_CalibrateScreen();
void Display_Splashscreen();
void Display_DebugTemplate();
void Display_DriveTemplate();
void Display_Update();
void Drive_Display_Update();
void Debug_Display_Update();
void Error_Display_Update();

/* [] END OF FILE */


#endif /* INC_FRUCD_DISPLAY_H_ */
