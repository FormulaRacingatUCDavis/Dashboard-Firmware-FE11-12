#ifndef __CAN_SD_H__
#define __CAN_SD_H__

#include "fatfs.h"
//#include "can_manager.h"
extern CAN_HandleTypeDef hcan1;

extern FATFS SDFatFS;
extern FIL SDFile;
char buffer[8192];
FRESULT res; /* FatFs function common result code */
DWORD fileSize;
uint32_t byteswritten, bytesread; /* File write/read counts */

extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];

char filename[20] = "error.txt";


void bufclear(void){
	for(int i = 0; i<8192; i++){
		buffer[i] = '\0';
	}
}

void mount_sd_card(void) {
  	 res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
	           //Open file for writing (Create)

  	 // name the file, increment until filename that hasnt been taken
  	 int num = 65;
  	 while (1) {
  		  FIL F1;
  		  sprintf(filename, "run_%c.txt", (char)num);

  		  FRESULT f_open_status = f_open(&F1, filename, FA_READ);

  		  // if found filename thats not taken, use it
  		  if (f_open_status != FR_OK) {
  			  f_close(&F1);
  			  break;
  		  }

  		  // if current file more than 9000000 bytes, move onto next file
  		  // else, use current file
  		  if (f_size(&F1) >= 9000000) {
  			  num++;
  		  }
  		  else {
  			  break;
  		  }

  		  f_close(&F1);


  	 }

	 res = f_open(&SDFile, filename,  FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_WRITE);
	 res = f_close(&SDFile);
	 bufclear();
}

void write_rx_to_sd(void){
  int TimeRightAfter_Get_From_Slave = HAL_GetTick();

  res = f_open(&SDFile, filename,  FA_OPEN_APPEND | FA_WRITE);
  sprintf(buffer,"%lX, %d, %d, %d, %d, %d, %d, %d, %d, %d \n ", RxHeader.StdId, RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7], TimeRightAfter_Get_From_Slave );
  f_write(&SDFile, buffer, strlen((char *)buffer), (void *)&byteswritten);
  f_close(&SDFile);
  bufclear();
}

void write_tx_to_sd(CAN_TxHeaderTypeDef TxHeader, uint8_t TxData[]){
  int TimeRightAfter_Get_From_Slave = HAL_GetTick();

  res = f_open(&SDFile, filename,  FA_OPEN_APPEND | FA_WRITE);
  sprintf(buffer,"%lX, %d, %d, %d, %d, %d, %d, %d, %d, %d \n ", TxHeader.StdId, TxData[0], TxData[1], TxData[2], TxData[3], RxData[4], RxData[5], RxData[6], RxData[7], TimeRightAfter_Get_From_Slave );
  f_write(&SDFile, buffer, strlen((char *)buffer), (void *)&byteswritten);
  f_close(&SDFile);
  bufclear();
}

void sd_card_close_file() {
//	f_close(&SDFile);
}

#endif
