#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "serial.h"
#include "st7789.h"
#include "board.h"
#include "uart.h"
#include "mpu6050.h"
#include "hc12.h"

// === Configuration Constants ===
#define SERIAL_BAUDRATE       115200
#define I2C_CLOCK_SPEED       100000
#define LED_BLINK_DELAY_MS     20
#define ERROR_BLINK_DELAY_MS   150
#define HC12_AT_RESPONSE_SIZE 32

// === Pin Definitions ===
#define LED_PIN               13
#define LED_PORT              GPIOC

int main(void) {
    // Initialize system
    delay_init();
    serial_stdio_init(SERIAL_BAUDRATE);
    
    // Initialize HC-12 module
    HC12_Init();
    char resp[HC12_AT_RESPONSE_SIZE];
    HC12_EnterATMode();
    HC12_SendATCommand("AT+RX", resp, sizeof(resp));
    HC12_LeaveATMode();
    
    // Initialize LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    LED_PORT->MODER &= ~(3u << (LED_PIN * 2));
    LED_PORT->MODER |= (1u << (LED_PIN * 2));
    
    // Initialize I2C for MPU6050
    i2c1_init_100k(I2C_CLOCK_SPEED);

    // Initialize MPU6050
    if (mpu6050_init() < 0) {
        printf("Failed to initialize MPU6050\n");
        // Error indication with LED blinking
        for (;;) {
            LED_PORT->ODR ^= (1u << LED_PIN);
            delay_ms(ERROR_BLINK_DELAY_MS);
        }
    }

    printf("MPU6050 initialized successfully\n");
    printf("Sending binary data packets...\n");

    while (1) {
        mpu6050_raw_t sensor_data;
        mpu_data_packet_t data_packet;

        if (mpu6050_read_all(&sensor_data) == 0) {
            // Create binary data packet
            HC12_CreateDataPacket(&data_packet, 
                                sensor_data.ax, sensor_data.ay, sensor_data.az,
                                sensor_data.gx, sensor_data.gy, sensor_data.gz);
            
            // Send binary packet
            HC12_SendDataPacket(&data_packet);
            
            // Debug output (optional)
            printf("Packet sent: ax=%d, ay=%d, az=%d, gx=%d, gy=%d, gz=%d\n",
                   sensor_data.ax, sensor_data.ay, sensor_data.az,
                   sensor_data.gx, sensor_data.gy, sensor_data.gz);

        } else {
            printf("Failed to read from MPU6050\n");
        }

        // LED heartbeat
        LED_PORT->ODR ^= (1u << LED_PIN);
        delay_ms(LED_BLINK_DELAY_MS);
    }
}
