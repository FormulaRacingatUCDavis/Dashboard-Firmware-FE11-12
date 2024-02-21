/* ========================================
 *
 * APIs for updating FRUCD Dashboard display
 *
 * ========================================
*/
#include "frucd_display.h"

// Just a test function that displays elements at the supposed corners of the screen
void calibrateScreen() {
    UG_FillScreen(C_WHITE);
    //libTFT_FillRectangle(0, 0, 10, 10, RED);
    UG_FillFrame(0, 0, 10, 10, C_RED);
    //libTFT_FillRectangle(0, 262, 10, 10, BLUE);
    UG_FillFrame(0, 262, 10, 272, C_BLUE);
    //libTFT_FillRectangle(470, 0, 10, 10, GREEN);
    UG_FillFrame(470, 0, 480, 10, C_GREEN);
    //libTFT_FillRectangle(470, 262, 10, 10, YELLOW);
    UG_FillFrame(470, 262, 480, 272, C_YELLOW);
    CyDelay(300);
}

// Initializes dashboard layout/label
/*
void initDashTemplate() {
    UG_FillScreen(C_BLACK);
    UG_PutString(68, 10, "PACK SOC");
    UG_PutString(297, 10, "MAX PACK T");
    UG_PutString(10, 195, "STATE:");
    UG_PutString(10, 240, "GLV V:");
    UG_PutString(339, 195, "MC T:");
    UG_PutString(300, 240, "MOTOR T:");
}
*/

void debugTemplate(){
    UG_FillScreen(C_BLACK);
    UG_PutString(10, 10, "PACK SOC:");
    UG_PutString(250, 10, "MAX PACK T:");
    UG_PutString(10, 75, "STATE:");
    UG_PutString(250, 75, "MC T:");
    UG_PutString(10, 140, "GLV V:");
    UG_PutString(250, 140, "MOTOR T:");
    UG_PutString(10, 205, "SHUTDOWN:");
    UG_PutString(250, 205, "MC FAULT:");
}
void driveTemplate(){
    UG_FillScreen(C_BLACK);
    UG_PutString(68, 10, "PACK SOC");
    UG_PutString(297, 10, "MAX PACK T"); 
    UG_PutString(5, 180, "STATE:");
    UG_PutString(275, 180, "GLV V:");
    
}
/*
 * BEGIN: functions to show data on dash
 * - handle variety of digit counts
 * - color decision logic is in here
 */



/* Left in non-generalized functions for future reference*/


/*
void disp_SOC(uint8_t data) {
    UG_FontSelect(&FONT_32X53);
    UG_COLOR color;
    if (data == 100) {
        // handle 100% case
        color = C_GREEN;
        if (color != last_SOC_color) {
            // only draw rectangle if color changed
            UG_FillFrame(30, 35, 210, 170, color);
            last_SOC_color = color;
        }
        UG_PutColorString(55, 80, "100%", C_BLACK, C_GREEN);
    } else {
        // handle other cases
        char data_s[4];
        sprintf(data_s, "%d", data);
        if (data >= 10) {
            // handle 2 digit cases
            // choose color
            if (data > 75) {
                color = C_GREEN;
            } else if (data > 50) {
                color = C_YELLOW;
            } else if (data > 25) {
                color = C_ORANGE;
            } else {
                color = C_RED;
            }
            data_s[2] = '%';
            data_s[3] = '\0';
            if (color != last_SOC_color) {
                // only draw rectangle if color changed
                UG_FillFrame(30, 35, 210, 170, color);
                last_SOC_color = color;
            }
            UG_PutColorString(70, 80, data_s, C_BLACK, color);
        } else {
            // handle 1 digit cases
            data_s[1] = '%';
            data_s[2] = '\0';
            color = C_RED;
            
            
            
            
            //why is this here?
            UG_FillFrame(30, 35, 210, 170, C_BLACK);
            
            
            
            
            
            
            
            UG_FillFrame(30, 35, 210, 170, color);
            UG_PutColorString(90, 80, data_s, C_BLACK, color);
        }   
    }
    UG_FontSelect(&FONT_12X16);
}


*/


// size is big if 1, small if not
void disp_SOC(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size) {
    int stringSize;
    int xFont, yFont;
    int horizFontSize = 12 + 20*size;
    if (data >= 100) {
        stringSize = 4;
    }else if (data >= 10){
        stringSize = 3;
    }else{
        stringSize = 2;
    }
    
    xFont = (x1 + x2)/2 - ((horizFontSize*stringSize)/2);
    yFont = (y1 + y2)/2 - 5 - (size*17);    
    if(size == 1){
        UG_FontSelect(&FONT_32X53);
       
    }
        
    UG_COLOR color;
    
    char data_s[5];
    sprintf(data_s, "%d%%", data);
    
    if (data >= 100) {
        // handle 100% case
        color = C_GREEN;
    } else {
        if (data >= 10) {
            // handle 2 digit cases
            // choose color
            if (data > 75) {
                color = C_GREEN;
            } else if (data > 50) {
                color = C_YELLOW;
            } else if (data > 25) {
                color = C_ORANGE;
            } else {
                color = C_RED;
            }
        } else {
            color = C_RED;
            
        }   
    }
    if (color != last_SOC_color) {
    // only draw rectangle if color changed
        UG_FillFrame(x1, y1, x2, y2, color);
        last_SOC_color = color;
    }
    UG_PutColorString(xFont, yFont, data_s, C_BLACK, color);
    
    UG_FontSelect(&FONT_12X16);
}

/*

void disp_max_pack_temp(uint8_t data) {
    // choose color
    UG_COLOR color;
    if (data < 45) {
        color = C_GREEN;
    } else if (data < 50) {
        color = C_YELLOW;
    } else if (data < 55) {
        color = C_ORANGE;
    } else {
        color = C_RED;

        UG_FillFrame(270, 35, 450, 170, C_BLACK);
        UG_FillFrame(270, 35, 450, 170, color);
        
    }
    
    if (color != last_max_pack_temp_color) {
        // only draw rectangle if color changed
        UG_FillFrame(270, 35, 450, 170, color);
        last_max_pack_temp_color = color;
    }
    UG_FontSelect(&FONT_32X53);
    char data_s[4];
    sprintf(data_s, "%d", data);
    data_s[2] = 'C';
    data_s[3] = '\0';
    UG_PutColorString(310, 80, data_s, C_BLACK, color);
    UG_FontSelect(&FONT_12X16);
}

*/


void disp_max_pack_temp(uint8_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size) {
    int xFont, yFont;
    int horizFontSize = 12 + 20*size;
    int stringSize;
    stringSize = 3;
    xFont = (x1 + x2)/2 - ((horizFontSize*stringSize)/2);
    yFont = (y1 + y2)/2 - 5 - (size*17);    
    if(size == 1){
        UG_FontSelect(&FONT_32X53);
    }
    
    // choose color
    UG_COLOR color;
    if (data < 45) {
        color = C_GREEN;
    } else if (data < 50) {
        color = C_YELLOW;
    } else if (data < 55) {
        color = C_ORANGE;
    } else {
        color = C_RED; 
        
        UG_FillFrame(x1, y1, x2, y2, C_BLACK);
        UG_FillFrame(x1, y1, x2, y2, color);
    
    }

   
    
   
    if (color != last_max_pack_temp_color) {
        UG_FillFrame(x1, y1, x2, y2, color);
        last_max_pack_temp_color = color;
    }

    char data_s[4];
    sprintf(data_s, "%d C", data);
   // data_s[2] = 'C';
    //data_s[3] = '\0';
    UG_PutColorString(xFont, yFont, data_s, C_BLACK, color);
    UG_FontSelect(&FONT_12X16);
}


/*

void disp_state(uint8_t state) { // TODO
    UG_COLOR color;
    // check fault bit
    if (state & 0x80) {
        // *************** FAULTS ***************
        uint8 fault = state & 0x7f; // mask off fault bit
        switch(fault) {
            case NONE: // STARTUP (effectively)
                // not obtainable via CAN
                // would only show when hardcoded on startup
                color = C_YELLOW;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(95, 185, 240, 215, color);
                    last_state_color = color;
                }
                UG_PutColorString(100, 195, " STARTUP  ", C_BLACK, color);
                break;
            case DRIVE_REQUEST_FROM_LV:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(95, 185, 240, 215, color);
                    last_state_color = color;
                }
                UG_PutColorString(100, 195, "DRV FRM LV", C_BLACK, color);
                break;
            case CONSERVATIVE_TIMER_MAXED:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(95, 185, 240, 215, color);
                    last_state_color = color;
                }
                UG_PutColorString(100, 195, "PRE TM OUT", C_BLACK, color);
                break;
            case BRAKE_NOT_PRESSED:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(95, 185, 240, 215, color);
                    last_state_color = color;
                }
                UG_PutColorString(100, 195, "BR NOT PRS", C_BLACK, color);
                break;
            case HV_DISABLED_WHILE_DRIVING:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(95, 185, 240, 215, color);
                    last_state_color = color;
                }
                UG_PutColorString(100, 195, "HV OFF DRV", C_BLACK, color);
                break;
            case SENSOR_DISCREPANCY:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(95, 185, 240, 215, color);
                    last_state_color = color;
                }
                UG_PutColorString(100, 195, "SNSR DSCRP", C_BLACK, color);
                break;
            case BRAKE_IMPLAUSIBLE:
                color = C_YELLOW;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(95, 185, 240, 215, color);
                    last_state_color = color;
                }
                UG_PutColorString(100, 195, "BSPD TRIPD", C_BLACK, color); //whitespace to clear
                break;
            case ESTOP:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(95, 185, 240, 215, color);
                    last_state_color = color;
                }
                UG_PutColorString(100, 195, "SHTDWN OPN", C_BLACK, color); //whitespace to clear
                break;
        }
        
    } else {
        // *************** NO FAULTS ***************
        color = C_GREEN;
        if (color != last_state_color) {
            // only draw rectangle if color changed
            UG_FillFrame(95, 185, 240, 215, color);
            last_state_color = color;
        }
        switch(state) {
            case LV:
                UG_PutColorString(100, 195, "    LV    ", C_BLACK, color);
                break;
            case PRECHARGING:
                UG_PutColorString(100, 195, "PRECHARGE ", C_BLACK, color);
                break;
            case HV_ENABLED:
                UG_PutColorString(100, 195, "HV ENABLED", C_BLACK, color);
                break;
            case DRIVE:
                UG_PutColorString(100, 195, "  DRIVE   ", C_BLACK, color);
                break;
        }
    }
}


*/


void disp_state(uint8_t state, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size) { 
    int xFont, yFont;
    int horizFontSize = 12 + 20*size;
    int stringSize;
    stringSize = 10;
    xFont = (x1 + x2)/2 - ((horizFontSize*stringSize)/2);
    yFont = (y1 + y2)/2 - 5 - (size*17);    
    if(size == 1){
        UG_FontSelect(&FONT_32X53);
        
    }
    UG_COLOR color;
    // check fault bit
    if (state & 0x80) {
        // *************** FAULTS ***************
        uint8 fault = state & 0x7f; // mask off fault bit
        switch(fault) {
            case NONE: // STARTUP (effectively)
                // not obtainable via CAN
                // would only show when hardcoded on startup
                color = C_YELLOW;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, " STARTUP  ", C_BLACK, color);
                break;
            case DRIVE_REQUEST_FROM_LV:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "DRV FRM LV", C_BLACK, color);
                break;
            case CONSERVATIVE_TIMER_MAXED:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "PRE TM OUT", C_BLACK, color);
                break;
            case BRAKE_NOT_PRESSED:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "BR NOT PRS", C_BLACK, color);
                break;
            case HV_DISABLED_WHILE_DRIVING:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "HV OFF DRV", C_BLACK, color);
                break;
            case SENSOR_DISCREPANCY:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "SNSR DSCRP", C_BLACK, color);
                break;
            case BRAKE_IMPLAUSIBLE:
                color = C_YELLOW;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "BSPD TRIPD", C_BLACK, color); //whitespace to clear
                break;
            case SHUTDOWN_CIRCUIT_OPEN:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "SHTDWN OPN", C_BLACK, color); //whitespace to clear
                break;
            case UNCALIBRATED:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "UNCALIBRTD", C_BLACK, color); //whitespace to clear
                break;
            case HARD_BSPD:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "HARD BSPD", C_BLACK, color); //whitespace to clear
                break;
            case MC_FAULT:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "MC FAULT", C_BLACK, color); //whitespace to clear
                break;
            default:
                color = C_RED;
                if (color != last_state_color) {
                    // only draw rectangle if color changed
                    UG_FillFrame(x1, y1, x2, y2, color);
                    last_state_color = color;
                }
                UG_PutColorString(xFont, yFont, "YO WTF?", C_BLACK, color); //whitespace to clear
                break;
           
        }
    } else {
        // *************** NO FAULTS ***************
        color = C_GREEN;
        if (color != last_state_color) {
            // only draw rectangle if color changed
            UG_FillFrame(x1, y1, x2, y2, color);
            last_state_color = color;
        }
        switch(state) {
            case LV:
                UG_PutColorString(xFont, yFont, "    LV    ", C_BLACK, color);
                break;
            case PRECHARGING:
                UG_PutColorString(xFont, yFont, "PRECHARGE ", C_BLACK, color);
                break;
            case HV_ENABLED:
                UG_PutColorString(xFont, yFont, "HV ENABLED", C_BLACK, color);
                break;
            case DRIVE:
                UG_PutColorString(xFont, yFont, "  DRIVE   ", C_BLACK, color);
                break;
        }
    }
     UG_FontSelect(&FONT_12X16);
}

/*

void disp_glv_v(uint32_t data) {
    // translate from voltage divider measurement to true voltage
    // y = 0.4295x + 18.254
    data *= 859;
    data /= 2000; // 0.4295
    data += 18;
    UG_COLOR color;
    if (data > 1150) {
        color = C_GREEN;
    } else if (data > 1100) {
        color = C_YELLOW;
    } else if (data > 1050) {
        color = C_ORANGE;
    } else {
        color = C_RED;
    }
    if (color != last_glv_v_color) {
        // only draw rectangle if color changed
        UG_FillFrame(95, 230, 182, 260, color);
        last_glv_v_color = color;
    }
    char data_s[7];
    sprintf(data_s, "%d", data);
    // handle 3 digit cases; 2 and 1 are practically impossible
    if (data < 1000) {
        data_s[3] = data_s[2];
        data_s[2] = data_s[1];
        data_s[1] = data_s[0];  
        data_s[0] = ' ';
    }
    // shift over fractional part
    data_s[4] = data_s[3];
    data_s[3] = data_s[2];
    // add syntax
    data_s[2] = '.';
    data_s[5] = 'V';
    data_s[6] = '\0';
    UG_PutColorString(100, 240, data_s, C_BLACK, color);
}

*/


void disp_glv_v(uint32_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size) {   
    int xFont, yFont;
    int horizFontSize = 12 + 20*size;
    int stringSize;
    stringSize = 6;
    xFont = (x1 + x2)/2 - ((horizFontSize*stringSize)/2);
    yFont = (y1 + y2)/2 - 5 - (size*17);    
    
    
    if(size == 1){
        UG_FontSelect(&FONT_32X53);  
    }
    // translate from voltage divider measurement to true voltage
    // y = 0.4295x + 18.254
    data *= 859;
    data /= 2000; // 0.4295
    data += 18;
    UG_COLOR color;
    if (data > 1150) {
        color = C_GREEN;
    } else if (data > 1100) {
        color = C_YELLOW;
    } else if (data > 1050) {
        color = C_ORANGE;
    } else {
        color = C_RED;
    }
    if (color != last_glv_v_color) {
        // only draw rectangle if color changed
        UG_FillFrame(x1, y1, x2, y2, color);
        last_glv_v_color = color;
    }
    char data_s[7];
    sprintf(data_s, "%lu", data);
    // handle 3 digit cases; 2 and 1 are practically impossible
    if (data < 1000) {
        data_s[3] = data_s[2];
        data_s[2] = data_s[1];
        data_s[1] = data_s[0];  
        data_s[0] = ' ';
    }
    // shift over fractional part
    data_s[4] = data_s[3];
    data_s[3] = data_s[2];
    // add syntax
    data_s[2] = '.';
    data_s[5] = 'V';
    data_s[6] = '\0';
    UG_PutColorString(xFont, yFont, data_s, C_BLACK, color);
    UG_FontSelect(&FONT_12X16);
}


/*
void disp_mc_temp(uint16_t data) {
    if (data >= 100) {
        // handle 3 digit cases
        UG_FillFrame(410, 185, 470, 215, C_GREEN);
        char data_s[5];
        sprintf(data_s, "%d", data);
        data_s[3] = 'C';
        data_s[4] = '\0';
        UG_PutColorString(415, 195, data_s, C_BLACK, C_GREEN);
    } else {
        // handle 2 digit cases
        UG_FillFrame(410, 185, 470, 215, C_GREEN);
        char data_s[4];
        sprintf(data_s, "%d", data);
        data_s[2] = 'C';
        data_s[3] = '\0';
        UG_PutColorString(422, 195, data_s, C_BLACK, C_GREEN);
    }
}
*/

void disp_mc_temp(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size){
    disp_gen_temp(data, x1, y1, x2, y2, size);
}
/*
void disp_motor_temp(uint16_t data) {
    if (data >= 100) {
        // handle 3 digit cases
        UG_FillFrame(410, 230, 470, 260, C_GREEN);
        char data_s[5];
        sprintf(data_s, "%d", data);
        data_s[3] = 'C';
        data_s[4] = '\0';
        UG_PutColorString(415, 240, data_s, C_BLACK, C_GREEN);
    } else {
        // handle 2 digit cases
        UG_FillFrame(410, 230, 470, 260, C_GREEN);
        char data_s[4];
        sprintf(data_s, "%d", data);
        data_s[2] = 'C';
        data_s[3] = '\0';
        UG_PutColorString(422, 240, data_s, C_BLACK, C_GREEN);
    }
}
*/

void disp_motor_temp(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size){
    disp_gen_temp(data, x1, y1, x2, y2, size);
}



void disp_gen_temp(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size){
    int xFont, yFont;
    int horizFontSize = 12 + 20*size;
    int stringSize;
    if (data >= 100) {
        stringSize = 4;
    }else{
        stringSize = 3;
    }
    xFont = (x1 + x2)/2 - ((horizFontSize*stringSize)/2);
    yFont = (y1 + y2)/2 - 5 - (size*17);    
    if(size == 1){
        UG_FontSelect(&FONT_32X53);
        
    }
    if (data >= 100) {
        // handle 3 digit cases
        UG_FillFrame(x1, y1, x2, y2, C_GREEN);
        char data_s[5];
        sprintf(data_s, "%d", data);
        data_s[3] = 'C';
        data_s[4] = '\0';
        UG_PutColorString(xFont, yFont, data_s, C_BLACK, C_GREEN);
    } else {
        // handle 2 digit cases
        UG_FillFrame(x1, y1, x2, y2, C_GREEN);
        char data_s[4];
        sprintf(data_s, "%d", data);
        data_s[2] = 'C';
        data_s[3] = '\0';
        UG_PutColorString(xFont, yFont, data_s, C_BLACK, C_GREEN);
    }
     UG_FontSelect(&FONT_12X16);
}

void disp_gen_voltage(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size){
    int xFont, yFont;
    int horizFontSize = 12 + 20*size;
    int stringSize;
    stringSize = 6;
   
    xFont = (x1 + x2)/2 - ((horizFontSize*stringSize)/2);
    yFont = (y1 + y2)/2 - 5 - (size*17);    
    if(size == 1){
        UG_FontSelect(&FONT_32X53);
        
    }
    UG_FillFrame(x1, y1, x2, y2, C_GREEN);
    char data_s[7];
    sprintf(data_s, "%d", data);
    // handle 3 digit cases; 2 and 1 are practically impossible
    if (data < 1000) {
        data_s[3] = data_s[2];
        data_s[2] = data_s[1];
        data_s[1] = data_s[0];  
        data_s[0] = ' ';
    }
    // shift over fractional part
    data_s[4] = data_s[3];
    data_s[3] = data_s[2];
    // add syntax
    data_s[2] = '.';
    data_s[5] = 'V';
    data_s[6] = '\0';
    UG_PutColorString(xFont, yFont, data_s, C_BLACK, C_GREEN);
    
    UG_FontSelect(&FONT_12X16);
}

void disp_shutdown_circuit(uint8_t shutdown_flags, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size) {
    int xFont, yFont;
    int horizFontSize = 12 + 20*size;
    int stringSize;
    stringSize = 10;
    xFont = (x1 + x2)/2 - ((horizFontSize*stringSize)/2);
    yFont = (y1 + y2)/2 - 5 - (size*17);    
    if(size == 1){
        UG_FontSelect(&FONT_32X53);
        
    }
    UG_COLOR color;
    
    // 00| IMD_OK | BMS_OK | Shutdown Final | AIR1 | AIR2 | Precharge
    if (shutdown_flags & 0b00100000){
        color = C_RED;
        if (color != last_shutdown_color) {
            // only draw rectangle if color changed
            UG_FillFrame(x1, y1, x2, y2, color);
            last_shutdown_color = color;
        }
        UG_PutColorString(xFont, yFont, "IMD_OK", C_BLACK, color);
    }
    else if (shutdown_flags & 0b00010000) {
        color = C_RED;
        if (color != last_shutdown_color) {
            // only draw rectangle if color changed
            UG_FillFrame(x1, y1, x2, y2, color);
            last_shutdown_color = color;
        }
        UG_PutColorString(xFont, yFont, "BMS_OK", C_BLACK, color);
    }
    else if (shutdown_flags & 0b00001000) {
        color = C_RED;
        if (color != last_shutdown_color) {
            // only draw rectangle if color changed
            UG_FillFrame(x1, y1, x2, y2, color);
            last_shutdown_color = color;
        }
        UG_PutColorString(xFont, yFont, "Shutdown_Final", C_BLACK, color);
    }
    else if (shutdown_flags & 0b00000100) {
        color = C_RED;
        if (color != last_shutdown_color) {
            // only draw rectangle if color changed
            UG_FillFrame(x1, y1, x2, y2, color);
            last_shutdown_color = color;
        }
        UG_PutColorString(xFont, yFont, "AIR_1", C_BLACK, color);
    }
    else if (shutdown_flags & 0b00000010) {
        color = C_RED;
        if (color != last_shutdown_color) {
            // only draw rectangle if color changed
            UG_FillFrame(x1, y1, x2, y2, color);
            last_shutdown_color = color;
        }
        UG_PutColorString(xFont, yFont, "AIR_2", C_BLACK, color);
    }
    else if (shutdown_flags & 0b00000001) {
        color = C_RED;
        if (color != last_shutdown_color) {
            // only draw rectangle if color changed
            UG_FillFrame(x1, y1, x2, y2, color);
            last_shutdown_color = color;
        }
        UG_PutColorString(xFont, yFont, "Precharge", C_BLACK, color);
    }
    else {
        color = C_GREEN;
        if (color != last_shutdown_color) {
            // only draw rectangle if color changed
            UG_FillFrame(x1, y1, x2, y2, color);
            last_shutdown_color = color;
        }
        UG_PutColorString(xFont, yFont, "None", C_BLACK, color);
    }
    
    
    UG_FontSelect(&FONT_12X16);
}

void disp_debug(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size) {
    int xFont, yFont;
    int horizFontSize = 12 + 20*size;
    int stringSize;
    stringSize = 10;
    xFont = (x1 + x2)/2 - ((horizFontSize*stringSize)/2);
    yFont = (y1 + y2)/2 - 5 - (size*17);    
    if(size == 1){
        UG_FontSelect(&FONT_32X53);
    }
    
    // choose color
    UG_COLOR color = C_GREEN_YELLOW;

    char data_s[10];
    sprintf(data_s, "%d", data);
    UG_FillFrame(x1, y1, x2, y2, color);
    UG_PutColorString(xFont, yFont, data_s, C_BLACK, color);
    UG_FontSelect(&FONT_12X16);
}

void disp_mc_fault(uint8_t data[8], uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size) {
    int xFont, yFont;
    int horizFontSize = 12 + 20*size;
    int stringSize;
    stringSize = 10;
    xFont = (x1 + x2)/2 - ((horizFontSize*stringSize)/2);
    yFont = (y1 + y2)/2 - 5 - (size*17);    
    if(size == 1){
        UG_FontSelect(&FONT_32X53);
    }
    
    // choose color
    UG_COLOR color = C_GREEN;
    
    uint8_t printed_data = 0; 
    uint8_t i = 0;
    for (; i < 8; i++) {
        if (data[i] > 0) {
            printed_data = data[i];
            break;
        }
    }

    char data_s[10];
    sprintf(data_s, "%d:%d", i, printed_data);
    UG_FillFrame(x1, y1, x2, y2, color);
    UG_PutColorString(xFont, yFont, data_s, C_BLACK, color);
    UG_FontSelect(&FONT_12X16);
}

void clear_colors(void)
{
    last_SOC_color = C_BLACK;
    last_max_pack_temp_color = C_BLACK;
    last_state_color = C_BLACK;
    last_glv_v_color = C_BLACK;
    last_mc_temp_color = C_BLACK;
    last_motor_temp_color = C_BLACK;
    last_shutdown_color = C_BLACK;
}
    
/* [] END OF FILE */
