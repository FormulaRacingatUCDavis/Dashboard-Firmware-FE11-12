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
#define ENTRY_SIZE (4 + 8 + 4)

#define MAX_ASYNC_WRITES 3

// Uncomment this to check whether we've written the correct number of bytes
// #define SD_CARD_CHECK_WRITEOUT 1

extern FATFS SDFatFS;
extern FIL SDFile;

static char buffer[BUFLEN];
static uint32_t buffer_size = 0;

static SemaphoreHandle_t sd_mutex = NULL;
static StaticSemaphore_t sd_mutex_buffer;

/*
 * SD Card data is usually stored in .biscuit files,
 * which can be trivially converted to plaintext for further
 * analysis.
 */

typedef struct _BiscuitHeader_t {
	uint32_t magic; // Should equal 0xB155CC17
	uint16_t version; // Should equal 1 for now
	uint16_t padding; // Not needed
} BiscuitHeader_t;

static void sd_card_flush_internal(void);
static void sd_card_write_from_buffer(void);
static void sd_card_write_data_bytes(uint8_t* bytes, uint32_t count);

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

	  		  sprintf(filename, "run_%u.biscuit", num);

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
		 if (res == FR_OK) {
			 // Write the header
			 BiscuitHeader_t header = {
					 .magic = 0xB155CC17,
					 .version = 1,
					 .padding = 0
			 };
			 sd_card_write_data_bytes((uint8_t*)(&header), sizeof(header));
			 sd_card_write_from_buffer(); // Write out immediately
			 sd_card_flush_internal();
		 }
	}

	xSemaphoreGive(sd_mutex);

	return (res == FR_OK) ? SD_CARD_MOUNT_RESULT_SUCCESS : SD_CARD_MOUNT_RESULT_FAILED;
}

void sd_card_write_data_record(uint32_t id, uint8_t data[]) {
	uint32_t tick;

	xSemaphoreTake(sd_mutex, portMAX_DELAY);

	// make sure we don't reach the end of the buffer
	if ((buffer_size + ENTRY_SIZE) >= BUFLEN) {
		sd_card_write_from_buffer();
	}

	// Write the ids
	buffer[buffer_size] = id & 0xFF;
	buffer[buffer_size + 1] = (id >> 8) & 0xFF;
	buffer[buffer_size + 2] = (id >> 16) & 0xFF;
	buffer[buffer_size + 3] = (id >> 24) & 0xFF;

	// Now write the data
	memcpy(buffer + buffer_size + 4, data, 8);

	// Now write the tick
	tick = HAL_GetTick();
	buffer[buffer_size + 12] = tick & 0xFF;
	buffer[buffer_size + 13] = (tick >> 8) & 0xFF;
	buffer[buffer_size + 14] = (tick >> 16) & 0xFF;
	buffer[buffer_size + 15] = (tick >> 24) & 0xFF;

	buffer_size += ENTRY_SIZE;

	xSemaphoreGive(sd_mutex);
}

void sd_card_write_from_rx(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[]) {
	sd_card_write_data_record(rxHeader.StdId, rxData);
}

void sd_card_write_from_tx(CAN_TxHeaderTypeDef txHeader, uint8_t txData[]) {
	sd_card_write_data_record(txHeader.StdId, txData);
}

void sd_card_update_sync(void) {
	xSemaphoreTake(sd_mutex, portMAX_DELAY);

	sd_card_write_from_buffer();
	sd_card_flush_internal();

	xSemaphoreGive(sd_mutex);
}

void sd_card_update_async(void) {
	static uint32_t writes_since_flush = 0;

	xSemaphoreTake(sd_mutex, portMAX_DELAY);

	sd_card_write_from_buffer();
	++writes_since_flush;

	/* If we've gone too long without syncing, force a flush */
	if (writes_since_flush == MAX_ASYNC_WRITES) {
		sd_card_flush_internal();
		writes_since_flush = 0;
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
}

static void sd_card_write_data_bytes(uint8_t* bytes, uint32_t count) {
	// make sure we don't reach the end of the buffer
	if ((buffer_size + count) >= BUFLEN) {
		sd_card_write_from_buffer();
	}

	memcpy(buffer, bytes, count);
	buffer_size += count;
}

static void sd_card_write_from_buffer(void) {
	static UINT bytes_written = 0;

	if (buffer_size == 0) return;

	f_write(&SDFile, buffer, buffer_size, &bytes_written);

#ifdef SD_CARD_CHECK_WRITEOUT
	if (bytes_written != buffer_size) {
		// TODO: mark error
	}
#endif

	buffer_size = 0;

}
