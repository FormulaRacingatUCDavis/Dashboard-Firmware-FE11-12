#include "serial_print.h"

void print(char *str) {
	CDC_Transmit_HS((uint8_t *)str, strlen(str));
}

void dump_can_data() {
	print("insert CAN data here\n");
}
