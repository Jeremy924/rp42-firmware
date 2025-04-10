#ifndef PC_INTERFACE_H
#define PC_INTERFACE_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "usbd_cdc_if.h"

#define WRITE_COM(message) CDC_Transmit_FS(message, sizeof(message))


_Noreturn void run_console();
uint32_t Read(char* buf, uint32_t cap);
void Write(char* buf, uint32_t size);
void WriteString(char* buf);
void _WriteString(char* buf, uint8_t important);

#endif
