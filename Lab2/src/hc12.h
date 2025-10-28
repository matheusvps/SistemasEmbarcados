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

// === Data Protocol ===
#define DATA_PACKET_MAGIC    0xAA55        // Magic number to identify our packets
#define DATA_PACKET_SIZE     15            // Total packet size in bytes
#define DATA_PAYLOAD_SIZE    12            // Payload size (6 int16_t values)

// === Data Structure ===
typedef struct {
    uint16_t magic;         // Magic number for identification
    int16_t ax, ay, az;    // Accelerometer data
    int16_t gx, gy, gz;    // Gyroscope data
    uint8_t checksum;      // Simple checksum for data integrity
} mpu_data_packet_t;

// === API ===
void HC12_Init(void);
void HC12_SendByte(uint8_t byte);
void HC12_SendString(const char *str);
uint8_t HC12_ReceiveByte(uint8_t *byte);
uint32_t HC12_ReceiveString(char *buffer, uint32_t max_len);
void HC12_SendATCommand(const char *cmd, char *response, uint32_t resp_size);
void HC12_EnterATMode();
void HC12_LeaveATMode();

// === Binary Data Functions ===
uint8_t HC12_CalculateChecksum(const mpu_data_packet_t *packet);
void HC12_CreateDataPacket(mpu_data_packet_t *packet, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
void HC12_SendDataPacket(const mpu_data_packet_t *packet);
uint8_t HC12_ReceiveDataPacket(mpu_data_packet_t *packet);

#endif
