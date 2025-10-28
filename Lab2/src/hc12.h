#ifndef HC12_H
#define HC12_H

#include "stm32f4xx.h"
#include <stdint.h>

// === Configuration ===
#define HC12_USART           USART2        // Change as needed
#define HC12_BAUDRATE        9600
#define HC12_TIMEOUT_MS      100
#define HC12_SET_PORT        GPIOA
#define HC12_SET_PIN         1

// === API ===
void HC12_Init(void);
void HC12_SendByte(uint8_t byte);
void HC12_SendString(const char *str);
uint8_t HC12_ReceiveByte(uint8_t *byte);
uint32_t HC12_ReceiveString(char *buffer, uint32_t max_len);
void HC12_SendATCommand(const char *cmd, char *response, uint32_t resp_size);
void HC12_EnterATMode();
void HC12_LeaveATMode();

#endif
