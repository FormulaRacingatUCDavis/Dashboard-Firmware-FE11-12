#ifndef __CAN_SD_H__
#define __CAN_SD_H__

#include "fatfs.h"
#include "stm32f7xx_hal.h"
#include "semphr.h"

typedef enum _SD_CARD_MOUNT_RESULT {
	SD_CARD_MOUNT_RESULT_SUCCESS = 0,
	SD_CARD_MOUNT_RESULT_FAILED = 1,
} SD_CARD_MOUNT_RESULT;

SD_CARD_MOUNT_RESULT sd_card_mount(SemaphoreHandle_t mutex);
void sd_card_write_data_record(uint32_t id, uint8_t data[]);
void sd_card_write_from_rx(CAN_RxHeaderTypeDef rxHeader, uint8_t rxData[]);
void sd_card_write_from_tx(CAN_TxHeaderTypeDef txHeader, uint8_t txData[]);
void sd_card_update_sync(void);
void sd_card_update_async(void);
void sd_card_flush(void);

#endif
