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

int main(void) {
    delay_init();
    serial_stdio_init(115200);

    HC12_Init();

    char resp[32];
    HC12_EnterATMode();
    HC12_SendATCommand("AT+RX", resp, sizeof(resp));    // Should respond "OK"
    HC12_LeaveATMode();
    HC12_SendString("Hello HC-12!\n");

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u<<(13*2));
    GPIOC->MODER |=  (1u<<(13*2));
    serial_stdio_init(115200);
    i2c1_init_100k(50000000u);

    if (mpu6050_init() < 0) {
        printf("failed to initiate mpu6050");
        for (;;) {
            GPIOC->ODR ^= (1u<<13);
            delay_ms(150);
        }
    }

    printf("ax,ay,az,gx,gy,gz,temp\n");

    while (1) {
        mpu6050_raw_t r;
        char outputBuffer[64];

        if (mpu6050_read_all(&r) == 0) {
            // float ax = mpu6050_accel_g(r.ax);
            // float ay = mpu6050_accel_g(r.ay);
            // float az = mpu6050_accel_g(r.az);
            //float gx = mpu6050_gyro_dps(r.gx);
            //float gy = mpu6050_gyro_dps(r.gy);
            //float gz = mpu6050_gyro_dps(r.gz);
            //float tc = mpu6050_temp_c(r.temp_raw);

            sprintf(outputBuffer, "%d,%d,%d,%d,%d,%d\n", r.ax, r.ay, r.az, r.gx, r.gy, r.gz);
            printf("%s", outputBuffer);
            HC12_SendString(outputBuffer);

        } else {
            printf("failed to read from mpu6050");
        }

        GPIOC->ODR ^= (1u<<13);
        delay_ms(20);
    }
}
