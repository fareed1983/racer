#ifndef __UART_H_
#define __UART_H_

#include <stdint.h>
#include <stdlib.h>

int uartInit(char *path);
int uartWriteCmd(int serialFd, uint8_t cmd, uint8_t *payload, uint16_t payloadLen);
int uartReadCmd(int serialFd, uint8_t *outBuf, size_t outBufSize);

#endif