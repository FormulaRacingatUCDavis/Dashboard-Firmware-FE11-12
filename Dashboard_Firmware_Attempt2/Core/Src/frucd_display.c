/*
 * frucd_display.c
 *
 *  Created on: Dec 4, 2023
 *      Author: leoja
 */


/* ========================================
 *
 * APIs for updating FRUCD Dashboard display
 *
 * ========================================
*/
#include "frucd_display.h"
#include "can_manager.h"
#include "fsm.h"

extern volatile uint32_t front_right_wheel_speed;
extern volatile uint32_t front_left_wheel_speed;

typedef struct {
    uint16_t box_x1;
    uint16_t box_y1;
    uint16_t box_x2;
    uint16_t box_y2;
    uint16_t grn_ylw_cutoff;
    uint16_t ylw_org_cutoff;
    uint16_t org_red_cutoff;
    UG_FONT font;
    UG_COLOR last_color;
    uint16_t last_value;
    char units;
} TEXTBOX_CONFIG;



TEXTBOX_CONFIG soc_box = {
		.grn_ylw_cutoff = 75,
		.ylw_org_cutoff = 50,
		.org_red_cutoff = 25,
		.units = '%'};

TEXTBOX_CONFIG speed_box;

TEXTBOX_CONFIG bms_temp_box = {
		.grn_ylw_cutoff = 30,
		.ylw_org_cutoff = 40,
		.org_red_cutoff = 50,
		.units = 'C'};

TEXTBOX_CONFIG state_box;

TEXTBOX_CONFIG shutdown_box;

TEXTBOX_CONFIG mc_temp_box = {
		.grn_ylw_cutoff = 30,
		.ylw_org_cutoff = 40,
		.org_red_cutoff = 50,
		.units = 'C'
};

TEXTBOX_CONFIG motor_temp_box;

TEXTBOX_CONFIG mc_fault_state_box;

TEXTBOX_CONFIG glv_v_box = {
		.grn_ylw_cutoff = 1150,
		.ylw_org_cutoff = 1100,
		.org_red_cutoff = 1050,
		.units = 'V'};

bool debug_mode = false;
UG_GUI gui1963;


/// PRIVATE FUNCTION PROTOTYPES ///
void draw_speed(uint32_t fl_wheel_speed, uint32_t fr_wheel_speed);
void draw_soc(uint16_t soc);
void draw_bms_temp(uint16_t temp);
void draw_state(uint8_t state, uint16_t bms_status);
void draw_glv_v(int16_t glv_v);
void draw_mc_fault_state(uint8_t mc_fault_state);
void draw_motor_temp(uint16_t motor_temp);
void draw_mc_temp(uint16_t mc_temp);
void draw_shutdown(uint8_t shutdown);
void draw_value_textbox(TEXTBOX_CONFIG* cfg, uint16_t value);
void draw_textbox(TEXTBOX_CONFIG* cfg, UG_COLOR color, char* string, uint8_t str_len);
UG_COLOR value_to_color(TEXTBOX_CONFIG cfg, uint16_t value);


void Display_Init()
{
	SSD1963_Init();

	// Initialize global structure and set PSET to this.PSET.
	UG_Init(&gui1963, SSD1963_PSet, DISPLAY_WIDTH, DISPLAY_HEIGHT);

	UG_FontSetVSpace(0);
	UG_FontSetHSpace(0);

	// Register acceleratos.
	UG_DriverRegister(DRIVER_FILL_FRAME, (void*)HW_FillFrame);
	UG_DriverRegister(DRIVER_DRAW_LINE, (void*)HW_DrawLine);
	UG_DriverRegister(DRIVER_DRAW_IMAGE, (void*)HW_DrawImage);
}


// Just a test function that displays elements at the supposed corners of the screen
void Display_CalibrateScreen() {
    UG_FillScreen(C_WHITE);
    //UG_FillFrame(0, 0, 10, 10, C_RED);
    UG_FillFrame(0, 262, 10, 272, C_BLUE);
    UG_FillFrame(470, 0, 480, 10, C_GREEN);
    UG_FillFrame(470, 262, 480, 272, C_YELLOW);
}

void Display_DebugTemplate()
{
    debug_mode = true;

    // clear screen
    UG_FillScreen(C_BLACK);

    // draw labels
    UG_PutString(270, 0, "PACK SOC:");
    UG_PutString(30, 120, "MAX PACK T:");
//    UG_PutString(10, 75, "STATE:");
    UG_PutString(30, 180, "STATE:");
    UG_PutString(30, 0, "MC T:");
//    UG_PutString(10, 140, "GLV V:");
    UG_PutString(270, 180, "GLV V:");
    UG_PutString(30, 60, "MOTOR T:");
    UG_PutString(270, 60, "SHUTDOWN:");
    UG_PutString(270, 120, "MC FAULT:");

    // setup textbox configs
    soc_box.box_x1 = 270;
    soc_box.box_y1 = 20;
    soc_box.box_x2 = 450;
    soc_box.box_y2 = 50;
    soc_box.font = FONT_12X16;
    soc_box.last_color = C_BLACK;  // force box redraw
    soc_box.last_value = 255;

    shutdown_box.box_x1 = 270;
    shutdown_box.box_y1 = 80;
    shutdown_box.box_x2 = 450;
    shutdown_box.box_y2 = 110;
    shutdown_box.font = FONT_12X16;
    shutdown_box.last_color = C_BLACK;  // force box redraw
    shutdown_box.last_value = 255;

    //updated
    bms_temp_box.box_x1 = 30;
	bms_temp_box.box_y1 = 140;
	bms_temp_box.box_x2 = 210;
	bms_temp_box.box_y2 = 170;
	bms_temp_box.font = FONT_12X16;
	bms_temp_box.last_color = C_BLACK;  // force box redraw
	bms_temp_box.last_value = 255;

//	state_box.box_x1 = 10;
//	state_box.box_y1 = 100;
//	state_box.box_x2 = 240;
//	state_box.box_y2 = 130;
//	state_box.font = FONT_12X16;
//	state_box.last_color = C_BLACK;  // force box redraw

	//updated
	state_box.box_x1 = 30;
	state_box.box_y1 = 200;
	state_box.box_x2 = 210;
	state_box.box_y2 = 230;
	state_box.font = FONT_12X16;
	state_box.last_color = C_BLACK;  // force box redraw
	state_box.last_value = 255;


	//motor control temp updated
	mc_temp_box.box_x1 = 30;
	mc_temp_box.box_y1 = 20;
	mc_temp_box.box_x2 = 210;
	mc_temp_box.box_y2 = 50;
	mc_temp_box.font = FONT_12X16;
	mc_temp_box.last_color = C_BLACK;  // force box redraw
	mc_temp_box.last_value = 255;

	//updated
	motor_temp_box.box_x1 = 30;
	motor_temp_box.box_y1 = 80;
	motor_temp_box.box_x2 = 210;
	motor_temp_box.box_y2 = 110;
	motor_temp_box.font = FONT_12X16;
	motor_temp_box.last_color = C_BLACK;  // force box redraw
	motor_temp_box.last_value = 255;

	mc_fault_state_box.box_x1 = 270;
	mc_fault_state_box.box_y1 = 140;
	mc_fault_state_box.box_x2 = 450;
	mc_fault_state_box.box_y2 = 170;
	mc_fault_state_box.font = FONT_12X16;
	mc_fault_state_box.last_color = C_BLACK;  // force box redraw
	mc_fault_state_box.last_value = 255;


//	glv_v_box.box_x1 = 10;
//	glv_v_box.box_y1 = 165;
//	glv_v_box.box_x2 = 240;
//	glv_v_box.box_y2 = 195;
//	glv_v_box.font = FONT_12X16;
//	glv_v_box.last_color = C_BLACK;  // force box redraw

	//updated
	glv_v_box.box_x1 = 270;
	glv_v_box.box_y1 = 200;
	glv_v_box.box_x2 = 450;
	glv_v_box.box_y2 = 230;
	glv_v_box.font = FONT_12X16;
	glv_v_box.last_color = C_BLACK;  // force box redraw

}

void Display_DriveTemplate()
{
    debug_mode = false;

    // clear screen
    UG_FillScreen(C_BLACK);

    // draw labels
    UG_PutString(68, 10, "SPEED");
    UG_PutString(297, 10, "MAX PACK T");
    UG_PutString(30, 180, "STATE:");
    UG_PutString(275, 180, "GLV V:");

    // setup textbox configs
    soc_box.box_x1 = 270;
    soc_box.box_y1 = 35;
    soc_box.box_x2 = 450;
    soc_box.box_y2 = 95;
    soc_box.font = FONT_12X16;
    soc_box.last_color = C_BLACK;  // force box redraw
    soc_box.last_value = 255;

    speed_box.box_x1 = 30;
    speed_box.box_y1 = 35;
    speed_box.box_x2 = 210;
    speed_box.box_y2 = 170;
    speed_box.font = FONT_32X53;
    speed_box.last_color = C_BLACK;  // force box redraw
    speed_box.last_value = 255;

    bms_temp_box.box_x1 = 270;
	bms_temp_box.box_y1 = 110;
	bms_temp_box.box_x2 = 450;
	bms_temp_box.box_y2 = 170;
	bms_temp_box.font = FONT_12X16;
	bms_temp_box.last_color = C_BLACK;  // force box redraw
	bms_temp_box.last_value = 255;

	state_box.box_x1 = 30;
	state_box.box_y1 = 200;
	state_box.box_x2 = 210;
	state_box.box_y2 = 230;
	state_box.font = FONT_12X16;
	state_box.last_color = C_BLACK;  // force box redraw

	glv_v_box.box_x1 = 270;
	glv_v_box.box_y1 = 200;
	glv_v_box.box_x2 = 450;
	glv_v_box.box_y2 = 230;
	glv_v_box.font = FONT_12X16;
	glv_v_box.last_color = C_BLACK;  // force box redraw
}


void Display_Update()
{
//	soc = soc+1 ;
//	glv_v+=1;

	draw_speed(front_left_wheel_speed, front_right_wheel_speed);
    draw_soc(soc);
    draw_bms_temp(PACK_TEMP);
    draw_state(one_byte_state(), bms_status);
    draw_glv_v(glv_v);
}

void Debug_Display_Update() {
	draw_soc(soc);
	draw_bms_temp(PACK_TEMP);
	draw_state(one_byte_state(), bms_status);
	draw_glv_v(glv_v);
	draw_mc_fault_state(mc_fault);
	draw_motor_temp(motor_temp);
	draw_mc_temp(mc_temp);
	draw_shutdown(shutdown_flags);
}

void draw_speed(uint32_t fl_wheel_speed, uint32_t fr_wheel_speed) {
	draw_value_textbox(&speed_box, (fl_wheel_speed+fr_wheel_speed)/2);
}

void draw_soc(uint16_t soc)
{
	draw_value_textbox(&soc_box, soc);
}

void draw_bms_temp(uint16_t temp)
{
	draw_value_textbox(&bms_temp_box, temp);
}

void draw_mc_fault_state(uint8_t mc_fault_state){
	draw_value_textbox(&mc_fault_state_box, mc_fault_state);
}

void draw_motor_temp(uint16_t motor_temp) {
	float converted_motor_temp = motor_temp/10.0;
	UG_COLOR color;
	if (converted_motor_temp < 45) {
		color = C_GREEN;
	} else if (converted_motor_temp < 50) {
		color = C_YELLOW;
	} else {
		color = C_RED;
	}

	char str[11];
	sprintf(str, "%.1f", converted_motor_temp);
	draw_textbox(&motor_temp_box, color, str, 11);
}

void draw_mc_temp(uint16_t mc_temp) {
	float converted_mc_temp = mc_temp/10.0;
	UG_COLOR color;
	if (converted_mc_temp < 45) {
		color = C_GREEN;
	} else if (converted_mc_temp < 50) {
		color = C_YELLOW;
	} else {
		color = C_RED;
	}
	char str[11];
	sprintf(str, "%.1f", converted_mc_temp);
	draw_textbox(&mc_temp_box, color, str, 11);
}

void draw_shutdown(uint8_t shutdown) {

	char string[14];
	UG_COLOR color;
	color = C_GREEN;
	//the 1 is the bit you want to extract
	if (!(shutdown | 0b1)) {
		strcpy(string, "  PRECHARGE   ");
	}
	if(!(shutdown | 0b10)) {
		strcpy(string, "     AIR1     ");
	}
	if(!(shutdown | 0b100)) {
		strcpy(string, "     AIR2     ");
	}
	if(!(shutdown | 0b1000)) {
		strcpy(string, "SHUTDOWN FINAL");
	}
	if(!(shutdown | 0b10000)) {
		strcpy(string, "    BMS_OK    ");
	}
	if(!(shutdown | 0b100000)) {
		strcpy(string, "    IMD_OK    ");
	}
	else {
		strcpy(string, " NO SHUTDOWN ");
	}

	draw_textbox(&shutdown_box, color, string, 14);

}

void draw_state(uint8_t state, uint16_t bms_status)
{
    static uint8_t last_state = 255;
    static uint16_t last_bms_status;

    if((state == last_state) && (bms_status == last_bms_status))  // skip function if value is the same
    {
        return;
    }

    UG_COLOR color;
    char string[15];

    switch(bms_status)  // BMS faults more important than VCU faults
    {
        case PACK_TEMP_OVER:
        case PACK_TEMP_UNDER:
            color = C_RED;
            strcpy(string, " BMS TEMP ");
            break;
        case LOW_SOC:
            color = C_RED;
            strcpy(string, " LOW SOC ");
            break;
        case IMBALANCE:
            color = C_RED;
            strcpy(string, "IMBALANCE");
            break;
        case SPI_FAULT:
            color = C_RED;
            strcpy(string, "SPI FAULT");
            break;
        case CELL_VOLT_OVER:
            color = C_RED;
            strcpy(string, " OVERVOLT ");
            break;
        case CELL_VOLT_UNDER:
            color = C_RED;
            strcpy(string, "UNDERVOLT");
            break;
        default:
            // check fault bit
            if (state & 0x80) {
                // *************** FAULTS ***************
                uint8_t fault = state & 0x7f; // mask off fault bit
                switch(fault)
                {
//                    case NONE: // STARTUP (effectively)
                	case 255:
                        // not obtainable via CAN
                        // would only show when hardcoded on startup
                        color = C_YELLOW;
                        strcpy(string, " STARTUP  ");
                        break;
                    case DRIVE_REQUEST_FROM_LV:
                        color = C_RED;
                        strcpy(string, "DRV FRM LV");
                        break;
                    case CONSERVATIVE_TIMER_MAXED:
                        color = C_RED;
                        strcpy(string, "PRE TM OUT");
                        break;
                    case BRAKE_NOT_PRESSED:
                        color = C_RED;
                        strcpy(string, "BR NOT PRS");
                        break;
                    case HV_DISABLED_WHILE_DRIVING:
                        color = C_RED;
                        strcpy(string, "HV OFF DRV");
                        break;
                    case SENSOR_DISCREPANCY:
                        color = C_RED;
                        strcpy(string, "SNSR DSCRP");
                        break;
                    case BRAKE_IMPLAUSIBLE:
                        color = C_YELLOW;
                        strcpy(string, "BSPD TRIPD");
                        break;
                    case SHUTDOWN_CIRCUIT_OPEN:
                        color = C_RED;
                        strcpy(string, "SHTDWN OPN");
                        break;
                    case UNCALIBRATED:
                        color = C_RED;
                        strcpy(string, "UNCALIBRTD");
                        break;
                    case HARD_BSPD:
                        color = C_RED;
                        strcpy(string, "HARD BSPD");
                        break;
                    case MC_FAULT:
                        color = C_RED;
                        strcpy(string, " MC FAULT ");
                        break;
                    default:
                        color = C_RED;
                        strcpy(string, " YO WTF? ");
                        break;
                }
            }
            else
            {
                // *************** NO FAULTS ***************
                color = C_GREEN;
                switch(state)
                {
                    case LV:
                        strcpy(string, "    LV    ");
                        break;
                    case PRECHARGING:
                        strcpy(string, "PRECHARGE ");
                        break;
                    case HV_ENABLED:
                        strcpy(string, "HV ENABLED");
                        break;
                    case DRIVE:
                        strcpy(string, "  DRIVE   ");
                        break;
                    default:
						color = C_RED;
						strcpy(string, " YO WTF? ");
						break;
                }
            }
    }

    draw_textbox(&state_box, color, string, 11);
}


void draw_glv_v(int16_t data) {
	float converted_glv_v = data/100.0;
    UG_COLOR color;
    if (converted_glv_v > 10) {
        color = C_GREEN;
    } else if (converted_glv_v > 9) {
        color = C_YELLOW;
    } else {
    	color = C_RED;
    }

    char str[11];
    sprintf(str, "%.2f", converted_glv_v);
    draw_textbox(&glv_v_box, color, str, 11);
}



UG_COLOR value_to_color(TEXTBOX_CONFIG cfg, uint16_t value)
{
    if(cfg.grn_ylw_cutoff > cfg.ylw_org_cutoff)   // green for large red for small
    {
        if(value > cfg.grn_ylw_cutoff)
        {
            return C_GREEN;
        }
        else if(value > cfg.ylw_org_cutoff)
        {
            return C_YELLOW;
        }
        else if(value > cfg.org_red_cutoff)
        {
            return C_ORANGE_RED;  // normal orange looks yellow
        }
        else
        {
            return C_RED;
        }
    }
    else  // red for large green for small
    {
        if(value > cfg.org_red_cutoff)
        {
            return C_RED;
        }
        else if(value > cfg.ylw_org_cutoff)
        {
            return C_ORANGE_RED;  // normal orange looks yellow
        }
        else if(value > cfg.grn_ylw_cutoff)
        {
            return C_YELLOW;
        }
        else
        {
            return C_GREEN;
        }
    }
}

void draw_value_textbox(TEXTBOX_CONFIG* cfg, uint16_t value)
{
	if(value == cfg->last_value)
	{
		return;
	}

	cfg->last_value = value;

	UG_COLOR color = value_to_color(*cfg, value);
	char string[10];
	uint16_t str_len = sprintf(string, "%d%c", value, cfg->units);

	draw_textbox(cfg, color, string, str_len);
}

void draw_textbox(TEXTBOX_CONFIG* cfg, UG_COLOR color, char* string, uint8_t str_len)
{
    // determine x and y coordinates to center text
    uint16_t text_x = (cfg->box_x2 + cfg->box_x1)/2 - ((str_len * cfg->font.char_width)/2);
    uint16_t text_y = (cfg->box_y2 + cfg->box_y1)/2 - (cfg->font.char_height/2);

    if(color != cfg->last_color)
    {
        UG_FillFrame(cfg->box_x1, cfg->box_y1, cfg->box_x2, cfg->box_y2, color);
        cfg->last_color = color;
    }

    UG_FontSelect(&cfg->font);
    UG_PutColorString(text_x, text_y, string, C_BLACK, color);
}


/*
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
*/
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
/*

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
*/

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

/*
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
*/

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
/*
void disp_mc_temp(uint16_t data, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size){
    disp_gen_temp(data, x1, y1, x2, y2, size);
}
*/
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
/*
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
*/

/* [] END OF FILE */
