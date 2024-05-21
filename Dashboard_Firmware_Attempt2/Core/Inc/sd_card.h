#ifndef __CAN_SD_H__
#define __CAN_SD_H__

#include "fatfs.h"
#include "stm32f7xx_hal.h"

int mount_sd_card(void);
void write_rx_to_sd(void);
void write_tx_to_sd(CAN_TxHeaderTypeDef TxHeader, uint8_t TxData[]);
void sd_card_write(void);

#endif
