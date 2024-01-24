/*
 * ugui_SSD1963.c
 *
 *  Created on: Dec 4, 2023
 *      Author: leoja
 */


/*******************************************************************************
* File Name: ugui_SSD1963.c
*
* Description:
*  This is a driver for the uGui graphical library developed by
*  Achim Döbler.
*  It is for SSD1963 graphic controller found in a lot of low cost graphics
*  chinese displays an to be used with PSoC microcontrollers from Cypress.
*  Will test it with other PSoC micros as soon as I can.
*
* Note:
*  For more information about uGui...
*  Website: http://www.embeddedlightning.com/ugui/
*  Git: https://github.com/achimdoebler/UGUI
*  Forum: http://www.embeddedlightning.com/forum/
*  PDF Reference manual (excellent): http://www.embeddedlightning.com/download/reference-guide/?wpdmdl=205
*
*  Thanks to Achim Döbler for such a god job.
*
* Log version:
*  1.0 - June, 2018.       First version.
*
********************************************************************************
* Copyright (c) 2018 Andres F. Navas
* This driver follows the same license than the uGui library.
*******************************************************************************/

#include "ugui.h"
#include "ugui_SSD1963.h"
#include "stdint.h"
#include "stm32f7xx_hal.h"

#define FMC_DELAY_CYCLES 20

/* *** Glogal GUI structure for this. *** */

UG_GUI gui1963;

#define HDP (DISPLAY_WIDTH - 1)
#define VDP (DISPLAY_HEIGHT - 1)


#define LCD_REG              (*((volatile uint8_t *) 0x60000000)) 	/* RS = 0 */
#define LCD_RAM              (*((volatile uint8_t *) 0x60000001)) 	/* RS = 1 */


void Display_WriteCommand(uint8_t index)
{
	LCD_REG	= index;
	for(uint16_t i = 0; i<FMC_DELAY_CYCLES; i++);
}


void Display_WriteData(uint8_t data)
{
	LCD_RAM = data;
	for(uint16_t i = 0; i<FMC_DELAY_CYCLES; i++);
}


uint16_t Display_ReadData(void)
{
	return LCD_RAM;
}

void Display_WriteMultiData(uint8_t *data, uint16_t size)
{
    for(int16_t i = 0; i < size; i++)
    {
    	LCD_RAM = data[i];
    	for(uint16_t j = 0; j<FMC_DELAY_CYCLES; j++);
    }
}

void Display_Reset()
{
    Display_WriteCommand(0x01);         //Software reset
    //HAL_Delay(10);
}

void Display_Init()
{
    //int i;

    //HAL_Delay(100);

    /*
    for(i = 0; i < 5; i++)              // Hardware reset
    {
        D_RST_Write(0);
		HAL_Delay(50);
        D_RST_Write(1);
        HAL_Delay(50);
	}
	*/

    Display_Reset();                    //Software reset
    //HAL_Delay(10);
    Display_Reset();                    //Software reset
    //HAL_Delay(10);
    Display_Reset();                    //Software reset
    //HAL_Delay(10);

    //Display_WriteCommand(0xe2);         //set multiplier and divider of PLL
    //Display_WriteData(0x23);            //PLL = 120MHz
    //Display_WriteData(0x02);
    //Display_WriteData(0x04);

    Display_WriteCommand(0xe0);
    Display_WriteData(0x01);            //Enable PLL
    //HAL_Delay(50);

    Display_WriteCommand(0xe0);
    Display_WriteData(0x03);            //Lock PLL
    //HAL_Delay(50);
    Display_Reset();                    //Software reset
    //HAL_Delay(50);

    Display_WriteCommand(0xb0);  //set LCD mode set TFT 18Bits mode

	Display_WriteData(0x08); //set TFT Mode - 0x0c
    Display_WriteData(0x80); //set TFT mode and hsync + vsync + DEN mode
    Display_WriteData(0x01); //set horizontal size = 480 - 1 hightbyte
    Display_WriteData(0xdf); //set horizontal size = 480 - 1 lowbyte
    Display_WriteData(0x01); //set vertical sive = 272 - 1 hightbyte
    Display_WriteData(0x0f); //set vertical size = 272 - 1 lowbyte
    Display_WriteData(0x00); //set even/odd line RGB seq

    Display_WriteCommand(0xf0); //set pixel data I/F format = 8 bit
    Display_WriteData(0x00);

    Display_WriteCommand(0x3a); //set RGB format = 6 6 6
    Display_WriteData(0x60);

    Display_WriteCommand(0xe6); //set PCLK freq = 4.94 MHz; pixel clock frequency
    Display_WriteData(0x01);    //02
    Display_WriteData(0x45);    //ff
    Display_WriteData(0x47);    //ff

    Display_WriteCommand(0xb4); //set HBP
    Display_WriteData(0x02); //set Hsync = 600
    Display_WriteData(0x0d);
    Display_WriteData(0x00);    //set HBP 68
    Display_WriteData(0x2b);
    Display_WriteData(0x28);    //set VBP 16
    Display_WriteData(0x00);    //Set Hsync start position
    Display_WriteData(0x00);
    Display_WriteData(0x00);    //set Hsync pulse subpixel start pos

    Display_WriteCommand(0xb6); //set VBP
    Display_WriteData(0x01);    //set Vsync total 360
    Display_WriteData(0x1d);
    Display_WriteData(0x00);    //set VBP = 19
    Display_WriteData(0x0c);
    Display_WriteData(0x09);    //set Vsync pulse 8
    Display_WriteData(0x00);    //set Vsync pulse start pos
    Display_WriteData(0x00);

    Display_WriteCommand(0x2a); //set column address
    Display_WriteData(0x00);    //set start column address 0
    Display_WriteData(0x00);
    Display_WriteData(0x01);    //set end column address = 479
    Display_WriteData(0xdf);

    Display_WriteCommand(0x2b); //set page address
    Display_WriteData(0x00);    //set start page address = 0
    Display_WriteData(0x00);
    Display_WriteData(0x01);    //set end column address = 479
    Display_WriteData(0x0f);

    Display_WriteCommand(0x13); //set normal mode
    Display_WriteCommand(0x38); //set normal mode
    Display_WriteCommand(0x29); //set display on

    // Initialize global structure and set PSET to this.PSET.
    UG_Init(&gui1963, Display_PSet, DISPLAY_WIDTH, DISPLAY_HEIGHT);

    // Register acceleratos.
    UG_DriverRegister(DRIVER_FILL_FRAME, (void*)HW_FillFrame);
    UG_DriverRegister(DRIVER_DRAW_LINE, (void*)HW_DrawLine);
    UG_DriverRegister(DRIVER_DRAW_IMAGE, (void*)HW_DrawImage);

}

void Display_WindowSet(unsigned int s_x,unsigned int e_x,unsigned int s_y,unsigned int e_y)
{
    uint8_t data[4];

    data[0] = ((s_x)>>8);                   //SET start column address
    data[1] = (s_x);
    data[2] = ((e_x)>>8);			        //SET end column address
    data[3] = (e_x);
	Display_WriteCommand(0x2a);		        //SET column address
    Display_WriteMultiData(data, 4);


    data[0] = ((s_y)>>8);                   //SET start row address
    data[1] = (s_y);
    data[2] = ((e_y)>>8);			        //SET end row address
    data[3] = (e_y);
	Display_WriteCommand(0x2b);		        //SET row address
    Display_WriteMultiData(data, 4);
}

void Display_PSet(UG_S16 x, UG_S16 y, UG_COLOR c)
{
    uint8_t data[3];

    data[0] = (c >> 16);
    data[1] = (c >> 8);
    data[2] = (c);

    if((x < 0) ||(x >= DISPLAY_WIDTH) || (y < 0) || (y >= DISPLAY_HEIGHT)) return;

    Display_WindowSet(x, x + 1, y, y + 1);
    Display_WriteCommand(0x2c);
    Display_WriteMultiData(data, 3);

}

UG_RESULT HW_FillFrame(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c)
{
    uint16_t loopx, loopy;
    uint8_t data[3];

    data[0] = (c >> 16);
    data[1] = (c >> 8);
    data[2] = (c);

    if((x1 < 0) ||(x1 >= DISPLAY_WIDTH) || (y1 < 0) || (y1 >= DISPLAY_HEIGHT)) return UG_RESULT_FAIL;
    if((x2 < 0) ||(x2 >= DISPLAY_WIDTH) || (y2 < 0) || (y2 >= DISPLAY_HEIGHT)) return UG_RESULT_FAIL;

    Display_WindowSet(x1,x2,y1,y2);

    Display_WriteCommand(0x2c);
    for (loopx = x1; loopx < x2 + 1; loopx++)
    {
        for (loopy = y1; loopy < y2 + 1; loopy++)
        {
            Display_WriteMultiData(data, 3);
        }
    }

    return UG_RESULT_OK;
}

UG_RESULT HW_DrawLine( UG_S16 x1 , UG_S16 y1 , UG_S16 x2 , UG_S16 y2 , UG_COLOR c )
{
    if((x1 < 0) ||(x1 >= DISPLAY_WIDTH) || (y1 < 0) || (y1 >= DISPLAY_HEIGHT)) return UG_RESULT_FAIL;
    if((x2 < 0) ||(x2 >= DISPLAY_WIDTH) || (y2 < 0) || (y2 >= DISPLAY_HEIGHT)) return UG_RESULT_FAIL;

    // If it is a vertical or a horizontal line, draw it.
    // If not, then use original drawline routine.
    if ((x1 == x2) || (y1 == y2))
    {
        HW_FillFrame(x1, y1, x2, y2, c);
        return UG_RESULT_OK;
    }

    return UG_RESULT_FAIL;
}

UG_RESULT HW_DrawImage(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, uint8_t *image, uint16_t pSize)
{

    if((x1 < 0) ||(x1 >= DISPLAY_WIDTH) || (y1 < 0) || (y1 >= DISPLAY_HEIGHT)) return UG_RESULT_FAIL;
    if((x2 < 0) ||(x2 >= DISPLAY_WIDTH) || (y2 < 0) || (y2 >= DISPLAY_HEIGHT)) return UG_RESULT_FAIL;

    Display_WindowSet(x1,x2,y1,y2);

    Display_WriteCommand(0x2c);
    Display_WriteMultiData(image, pSize*3);

    return UG_RESULT_OK;
}



/* [] END OF FILE */
