/*
 * sd_card.c
 *
 *  Created on: May 20, 2024
 *      Author: Abhineet
 */

#include "sd_card.h"
#include "stdio.h"
#include "semphr.h"

#define BUFLEN 8192
#define MAX_STRLEN 500

#define MAX_ASYNC_WRITES 3

extern FATFS SDFatFS;
extern FIL SDFile;

static char buffer[BUFLEN];
static uint32_t ind = 0;
static uint32_t index_top = 0;

static UINT bytes_written = 0;
static uint32_t last_write_index = 0;

static SemaphoreHandle_t sd_mutex = NULL;
static StaticSemaphore_t sd_mutex_buffer;

static uint32_t writes_since_flush = 0;

static void sd_card_flush_internal(void);
static void sd_card_write_buffer(void);

SD_CARD_MOUNT_RESULT sd_card_mount(void) {
	sd_mutex = sd_mutex ? sd_mutex : xSemaphoreCreateMutexStatic(&sd_mutex_buffer);
	if (!sd_mutex)
		return SD_CARD_MOUNT_RESULT_FAILED;

	xSemaphoreTake(sd_mutex, portMAX_DELAY);
	FRESULT res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
	if (res == FR_OK) {
		char filename[20];

		/* name the file, increment until filename hasn't been taken */
	  	 uint8_t num = 0;
	  	 while (1) {
	  		  FIL F1;

	  		  sprintf(filename, "run_%u.txt", num);

	  		  FRESULT f_open_status = f_open(&F1, filename, FA_READ);

	  		  /* if found filename thats not taken, use it */
	  		  if (f_open_status == FR_NO_FILE) {
	  			  f_close(&F1);
	  			  break;
	  		  }

	  		  ++num;
	  		  f_close(&F1);
	  	 }

		 res = f_open(&SDFile, filename,  FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_WRITE);
	}

	xSemaphoreGive(sd_mutex);

	return (res == FR_OK) ? SD_CARD_MOUNT_RESULT_SUCCESS : SD_CARD_MOUNT_RESULT_FAILED;
}

void sd_card_write_data_record(uint32_t id, uint8_t data[]) {
	UINT tick = HAL_GetTick();

	xSemaphoreTake(sd_mutex, portMAX_DELAY);

	// make sure we don't reach the end of the buffer
	if ((ind + MAX_STRLEN) >= BUFLEN) {
		index_top = ind;
		ind = 0;
	}

	int bytes = sprintf(&buffer[ind], "%lX,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",
			id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], tick);
	ind += bytes;

	xSemaphoreGive(sd_mutex);
}

void sd_card_write_from_rx(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[]) {
	sd_card_write_data_record(rxHeader.StdId, rxData);
}

void sd_card_write_from_tx(CAN_TxHeaderTypeDef txHeader, uint8_t txData[]) {
	sd_card_write_data_record(txHeader.StdId, txData);
}

void sd_card_write_sync(void) {
	xSemaphoreTake(sd_mutex, portMAX_DELAY);

	sd_card_write_buffer();
	sd_card_flush_internal();

	xSemaphoreGive(sd_mutex);
}

void sd_card_write_async(void) {
	xSemaphoreTake(sd_mutex, portMAX_DELAY);

	sd_card_write_buffer();

	/* If we've gone too long without syncing, force a flush */
	if (writes_since_flush == MAX_ASYNC_WRITES) {
		sd_card_flush_internal();
	}

	xSemaphoreGive(sd_mutex);
}

/* Public variant, locks mutex */
void sd_card_flush(void) {
	xSemaphoreTake(sd_mutex, portMAX_DELAY);
	sd_card_flush_internal();
	xSemaphoreGive(sd_mutex);
}

/* Only to be used if mutex is active. */
static void sd_card_flush_internal(void) {
	f_sync(&SDFile);
	writes_since_flush = 0;
}

static void sd_card_write_buffer(void) {
	if (ind == last_write_index) {
		return;
	} else if (ind > last_write_index) {
		f_write(&SDFile, buffer + last_write_index, (ind - last_write_index), &bytes_written);
	} else {
		/* index < last_write_index */
		f_write(&SDFile, buffer + last_write_index, (index_top - last_write_index), &bytes_written);
		f_write(&SDFile, buffer, ind, &bytes_written);
	}

	last_write_index = ind;
	++writes_since_flush;

	//TODO: compare bytes_written to expected value?
	//TODO: check results of write operations
}
