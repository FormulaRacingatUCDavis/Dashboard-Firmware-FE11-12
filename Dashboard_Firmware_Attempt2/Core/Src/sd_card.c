/*
 * sd_card.c
 *
 *  Created on: May 20, 2024
 *      Author: Abhineet
 */

#include "sd_card.h"
#include "stdio.h"

#define BUFLEN 8192
#define MAX_STRLEN 500

extern FATFS SDFatFS;
extern FIL SDFile;

char buffer[BUFLEN];
uint32_t ind = 0;
uint32_t index_top = 0;
UINT byteswritten;


extern CAN_HandleTypeDef hcan1;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];


int mount_sd_card(void) {
  	 FRESULT res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
  	 if (res != FR_OK) return 0;

  	 char filename[20];

  	 // name the file, increment until filename has'nt been taken
  	 uint8_t num = 0;
  	 while (1) {
  		  FIL F1;

  		  sprintf(filename, "run_%u.txt", num);

  		  FRESULT f_open_status = f_open(&F1, filename, FA_READ);

  		  // if found filename thats not taken, use it
  		  if (f_open_status == FR_NO_FILE) {
  			  f_close(&F1);
  			  break;
  		  }
  		  num++;

  		  f_close(&F1);
  	 }

	 res = f_open(&SDFile, filename,  FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_WRITE);
	 if(res == FR_OK){
		 return 1;
	 }
	 return 0;
}

void write_data_record(uint32_t id, uint8_t data[8]){
	int tick = HAL_GetTick();

	// make sure we don't reach the end of the buffer
	if((ind + MAX_STRLEN) >= BUFLEN){
		index_top = ind;
		ind = 0;
	}

	int bytes = sprintf(&buffer[ind], "%lX,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], tick);
	ind += bytes;
}

void write_rx_to_sd(void){
	write_data_record(RxHeader.StdId, RxData);
}

void write_tx_to_sd(CAN_TxHeaderTypeDef TxHeader, uint8_t TxData[]){
	write_data_record(TxHeader.StdId, TxData);
}

void sd_card_write(void){
	static uint32_t last_write_index = 0;

	if(ind == last_write_index){
		return;
	} else if(ind > last_write_index){
		f_write(&SDFile, buffer + last_write_index, (ind - last_write_index), &byteswritten);
	} else { // index < last_write_index
		f_write(&SDFile, buffer + last_write_index, (index_top - last_write_index), &byteswritten);
		f_write(&SDFile, buffer, ind, &byteswritten);
	}

	f_sync(&SDFile); // sync less often?
	last_write_index = ind;

	//TODO: compare byteswritten to expected value?
	//TODO: check results of write operations
}
