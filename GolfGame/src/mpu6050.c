    #include "mpu6050.h"
#include "stm32f4xx.h"
#include <stdio.h>

static int i2c_timeout(uint32_t *t) {
    if ((*t)-- == 0) return -1;
    return 0;
}

void i2c1_init_100k(uint32_t apb1_hz) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~((3u<<(8*2)) | (3u<<(9*2)));
    GPIOB->MODER |=  ((2u<<(8*2)) | (2u<<(9*2)));
    GPIOB->AFR[1]  &= ~((0xFu<<((8-8)*4)) | (0xFu<<((9-8)*4)));
    GPIOB->AFR[1]  |=  ((4u<<((8-8)*4))   | (4u<<((9-8)*4)));
    GPIOB->OTYPER |=  (1u<<8) | (1u<<9);
    GPIOB->PUPDR  &= ~((3u<<(8*2)) | (3u<<(9*2)));
    GPIOB->PUPDR  |=  ((1u<<(8*2)) | (1u<<(9*2)));
    GPIOB->OSPEEDR |= ((3u<<(8*2)) | (3u<<(9*2)));

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    I2C1->CR1 = 0;
    I2C1->CR2 = 0;

    uint32_t freq_mhz = apb1_hz / 1000000u;
    if (freq_mhz < 2) freq_mhz = 2;
    if (freq_mhz > 42) freq_mhz = 42;
    I2C1->CR2 = (uint16_t)freq_mhz;

    uint16_t ccr = (uint16_t)(apb1_hz / (2u * 100000u));
    if (ccr < 4) ccr = 4;
    I2C1->CCR = ccr & 0x0FFF;
    I2C1->TRISE = (uint16_t)(freq_mhz + 1u);
    I2C1->CR1 |= I2C_CR1_PE;
}

static int i2c1_start_addr(uint8_t addr7, int read) {
    uint32_t to = 1000000;
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) if (i2c_timeout(&to)) return -1;
    (void)I2C1->SR1;
    I2C1->DR = (addr7<<1) | (read?1:0);
    if (!read) {
        while (!(I2C1->SR1 & I2C_SR1_ADDR)) if (i2c_timeout(&to)) return -1;
        (void)I2C1->SR1; (void)I2C1->SR2;
    } else {
        while (!(I2C1->SR1 & I2C_SR1_ADDR)) if (i2c_timeout(&to)) return -1;
        (void)I2C1->SR1; (void)I2C1->SR2;
    }
    return 0;
}

static void i2c1_stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

int i2c1_write_reg(uint8_t addr7, uint8_t reg, uint8_t data) {
    uint32_t to = 1000000;
    if (i2c1_start_addr(addr7, 0) < 0) { i2c1_stop(); return -1; }
    while (!(I2C1->SR1 & I2C_SR1_TXE)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    i2c1_stop();
    return 0;
}

int i2c1_read_reg(uint8_t addr7, uint8_t reg, uint8_t *data) {
    uint32_t to = 1000000;
    if (i2c1_start_addr(addr7, 0) < 0) { i2c1_stop(); return -1; }
    while (!(I2C1->SR1 & I2C_SR1_TXE)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    (void)I2C1->SR1;
    I2C1->DR = (addr7<<1) | 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    I2C1->CR1 &= ~I2C_CR1_ACK;
    (void)I2C1->SR1; (void)I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE)) if (i2c_timeout(&to)) return -1;
    *data = (uint8_t)I2C1->DR;
    return 0;
}

int i2c1_read_multi(uint8_t addr7, uint8_t reg, uint8_t *buf, uint32_t len) {
    if (len == 0) return 0;
    uint32_t to = 1000000;
    if (i2c1_start_addr(addr7, 0) < 0) { i2c1_stop(); return -1; }
    while (!(I2C1->SR1 & I2C_SR1_TXE)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    (void)I2C1->SR1;
    I2C1->DR = (addr7<<1) | 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) if (i2c_timeout(&to)) { i2c1_stop(); return -1; }
    if (len == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR1; (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) if (i2c_timeout(&to)) return -1;
        buf[0] = (uint8_t)I2C1->DR;
        return 0;
    }
    I2C1->CR1 |= I2C_CR1_ACK;
    (void)I2C1->SR1; (void)I2C1->SR2;
    for (uint32_t i=0; i<len; i++) {
        if (i == (len-2)) {
            while (!(I2C1->SR1 & I2C_SR1_BTF)) if (i2c_timeout(&to)) return -1;
            I2C1->CR1 &= ~I2C_CR1_ACK;
            buf[i] = (uint8_t)I2C1->DR;
            I2C1->CR1 |= I2C_CR1_STOP;
            while (!(I2C1->SR1 & I2C_SR1_RXNE)) if (i2c_timeout(&to)) return -1;
            buf[i+1] = (uint8_t)I2C1->DR;
            break;
        } else {
            while (!(I2C1->SR1 & I2C_SR1_RXNE)) if (i2c_timeout(&to)) return -1;
            buf[i] = (uint8_t)I2C1->DR;
        }
    }
    return 0;
}

int mpu6050_init(void) {
    if (i2c1_write_reg(MPU6050_ADDR, MPU6050_REG_PWR1, 0x00) < 0) return -1;
    if (i2c1_write_reg(MPU6050_ADDR, MPU6050_REG_CONFIG, 0x03) < 0) return -1;
    if (i2c1_write_reg(MPU6050_ADDR, MPU6050_REG_GYROCFG, 0x00) < 0) return -1;
    if (i2c1_write_reg(MPU6050_ADDR, MPU6050_REG_ACCELCFG, 0x00) < 0) return -1;
    if (i2c1_write_reg(MPU6050_ADDR, MPU6050_REG_SMPLRT, 9)    < 0) return -1;
    return 0;
}

int mpu6050_read_all(mpu6050_raw_t *out) {
    uint8_t buf[14];
    if (i2c1_read_multi(MPU6050_ADDR, MPU6050_REG_ACCEL, buf, 14) < 0) return -1;
    out->ax = (int16_t)((buf[0]<<8) | buf[1]);
    out->ay = (int16_t)((buf[2]<<8) | buf[3]);
    out->az = (int16_t)((buf[4]<<8) | buf[5]);
    out->temp_raw = (int16_t)((buf[6]<<8) | buf[7]);
    out->gx = (int16_t)((buf[8]<<8) | buf[9]);
    out->gy = (int16_t)((buf[10]<<8) | buf[11]);
    out->gz = (int16_t)((buf[12]<<8) | buf[13]);
    return 0;
}

float mpu6050_accel_g(int16_t raw) {
    return raw / 16384.0f;
}

float mpu6050_gyro_dps(int16_t raw) {
    return raw / 131.0f;
}

float mpu6050_temp_c(int16_t raw) {
    return (raw/340.0f) + 36.53f;
}

