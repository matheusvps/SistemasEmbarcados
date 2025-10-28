#include "hc12.h"
#include "delay.h"

void HC12_Init(void) {
    // === Enable peripheral clocks ===
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // USART2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // GPIOA clock

    // === Configure PA2 (TX) and PA3 (RX) ===
    GPIOA->MODER &= ~((3 << (2*2)) | (3 << (3*2))); // clear mode
    GPIOA->MODER |= (2 << (2*2)) | (2 << (3*2));    // alternate function
    GPIOA->AFR[0] |= (7 << (2*4)) | (7 << (3*4));   // AF7 = USART2

    // === Configure USART2 ===
    HC12_USART->BRR = SystemCoreClock / HC12_BAUDRATE;  // APB1 = /2 prescaler
    HC12_USART->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    HC12_USART->CR1 &= ~USART_CR1_PCE;   // disable parity
    HC12_USART->CR1 &= ~USART_CR1_M;     // 8-bit word length
    HC12_USART->CR2 &= ~USART_CR2_STOP;  // 1 stop bit
}

void HC12_SendByte(uint8_t byte) {
    while (!(HC12_USART->SR & USART_SR_TXE));
    HC12_USART->DR = byte;
}

void HC12_SendString(const char *str) {
    while (*str) {
        HC12_SendByte((uint8_t)*str++);
    }
}

uint8_t HC12_ReceiveByte(uint8_t *byte) {
    uint32_t timeout = HC12_TIMEOUT_MS * (SystemCoreClock / 8000);
    while (!(HC12_USART->SR & USART_SR_RXNE)) {
        if (!timeout--) return 0;  // timeout
    }
    *byte = (uint8_t)(HC12_USART->DR & 0xFF);
    return 1;
}

uint32_t HC12_ReceiveString(char *buffer, uint32_t max_len) {
    uint32_t count = 0;
    uint8_t b;
    while (count < max_len - 1) {
        if (!HC12_ReceiveByte(&b)) break;
        if (b == '\n' || b == '\r') break;
        buffer[count++] = b;
    }
    buffer[count] = '\0';
    return count;
}

void HC12_SendATCommand(const char *cmd, char *response, uint32_t resp_size) {
    // The module must be in AT mode (SET pin LOW)
    HC12_SendString(cmd);
    HC12_SendString("\r\n");
    delay_ms(100);
    HC12_ReceiveString(response, resp_size);
}

void HC12_EnterATMode() {
    HC12_SET_PORT->BSRR = (1U << (HC12_SET_PIN + 16));  // reset bit = LOW
    delay_ms(80);
}

void HC12_LeaveATMode() {
    HC12_SET_PORT->BSRR = (1U << HC12_SET_PIN);         // set bit = HIGH
    delay_ms(80);
}

// === Binary Data Functions ===

uint8_t HC12_CalculateChecksum(const mpu_data_packet_t *packet) {
    uint8_t checksum = 0;
    const uint8_t *data = (const uint8_t*)packet;
    
    // Calculate checksum for all bytes except the checksum field itself
    for (int i = 0; i < sizeof(mpu_data_packet_t) - 1; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void HC12_CreateDataPacket(mpu_data_packet_t *packet, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
    packet->magic = DATA_PACKET_MAGIC;
    packet->ax = ax;
    packet->ay = ay;
    packet->az = az;
    packet->gx = gx;
    packet->gy = gy;
    packet->gz = gz;
    packet->checksum = HC12_CalculateChecksum(packet);
}

void HC12_SendDataPacket(const mpu_data_packet_t *packet) {
    const uint8_t *data = (const uint8_t*)packet;
    
    for (int i = 0; i < sizeof(mpu_data_packet_t); i++) {
        HC12_SendByte(data[i]);
    }
}

uint8_t HC12_ReceiveDataPacket(mpu_data_packet_t *packet) {
    uint8_t *data = (uint8_t*)packet;
    
    // Receive all bytes of the packet
    for (int i = 0; i < sizeof(mpu_data_packet_t); i++) {
        if (!HC12_ReceiveByte(&data[i])) {
            return 0; // Timeout or error
        }
    }
    
    // Verify magic number
    if (packet->magic != DATA_PACKET_MAGIC) {
        return 0; // Invalid packet
    }
    
    // Verify checksum
    uint8_t calculated_checksum = HC12_CalculateChecksum(packet);
    if (packet->checksum != calculated_checksum) {
        return 0; // Checksum error
    }
    
    return 1; // Success
}
