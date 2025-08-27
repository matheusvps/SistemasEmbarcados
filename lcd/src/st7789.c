#include "st7789.h"
#include "stm32f4xx.h"
#include <stdint.h>

// --- Pin configuration (adjust GPIO port/pin as needed) ---
#define ST7789_DC_PORT     GPIOA
#define ST7789_DC_PIN      0
#define ST7789_DC_PIN_BIT  (1U << ST7789_DC_PIN)

#define ST7789_RST_PORT    GPIOA
#define ST7789_RST_PIN     1
#define ST7789_RST_PIN_BIT (1U << ST7789_RST_PIN)

#define ST7789_CS_PORT     GPIOA
#define ST7789_CS_PIN      2
#define ST7789_CS_PIN_BIT  (1U << ST7789_CS_PIN)

// --- Macros ---
#define DC_COMMAND()       (ST7789_DC_PORT->BSRR = (ST7789_DC_PIN_BIT << 16))
#define DC_DATA()          (ST7789_DC_PORT->BSRR = (ST7789_DC_PIN_BIT))

#define RST_LOW()          (ST7789_RST_PORT->BSRR = (ST7789_RST_PIN_BIT << 16))
#define RST_HIGH()         (ST7789_RST_PORT->BSRR = ST7789_RST_PIN_BIT)

#define CS_LOW()           (ST7789_CS_PORT->BSRR = (ST7789_CS_PIN_BIT << 16))
#define CS_HIGH()          (ST7789_CS_PORT->BSRR = ST7789_CS_PIN_BIT)

static void ST7789_Write(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    *((__IO uint8_t*)&SPI1->DR) = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    (void)SPI1->DR;
}

static void ST7789_Command(uint8_t cmd) {
    DC_COMMAND();
    CS_LOW();
    ST7789_Write(cmd);
    CS_HIGH();
}

static void ST7789_Data(uint8_t data) {
    DC_DATA();
    CS_LOW();
    ST7789_Write(data);
    CS_HIGH();
}

static void ST7789_DataBuffer(uint8_t *buff, uint32_t len) {
    DC_DATA();
    CS_LOW();
    for (uint32_t i = 0; i < len; i++) {
        ST7789_Write(buff[i]);
    }
    CS_HIGH();
}

static void delay_ms(volatile uint32_t ms) {
    for (; ms > 0; ms--) {
        for (volatile uint32_t i = 0; i < 16000; i++);
    }
}

void ST7789_Init(void) {
    RST_LOW();
    delay_ms(50);
    RST_HIGH();
    delay_ms(50);

    ST7789_Command(0x36);   // MADCTL
    ST7789_Data(0x00);

    ST7789_Command(0x3A);   // COLMOD
    ST7789_Data(0x55);      // 16-bit/pixel

    ST7789_Command(0x21);   // Inversion ON

    ST7789_Command(0x11);   // Sleep OUT
    delay_ms(120);

    ST7789_Command(0x29);   // Display ON
    delay_ms(20);
}


void ST7789_GPIO_Init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // --- Configure PA5 = SPI1_SCK (AF5) ---
    GPIOA->MODER &= ~(3U << (5 * 2));      // clear mode
    GPIOA->MODER |=  (2U << (5 * 2));      // alternate function
    GPIOA->OSPEEDR |= (3U << (5 * 2));     // very high speed
    GPIOA->PUPDR &= ~(3U << (5 * 2));      // no pull
    GPIOA->AFR[0] &= ~(0xF << (5 * 4));
    GPIOA->AFR[0] |=  (5U << (5 * 4));     // AF5 = SPI1

    // --- Configure PA7 = SPI1_MOSI (AF5) ---
    GPIOA->MODER &= ~(3U << (7 * 2));
    GPIOA->MODER |=  (2U << (7 * 2));
    GPIOA->OSPEEDR |= (3U << (7 * 2));
    GPIOA->PUPDR &= ~(3U << (7 * 2));
    GPIOA->AFR[0] &= ~(0xF << (7 * 4));
    GPIOA->AFR[0] |=  (5U << (7 * 4));

    // --- Configure PA0 (DC), PA1 (RST), PA2 (CS) as outputs ---
    GPIOA->MODER &= ~((3U << (1 * 2)) | (3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA->MODER |=  ((1U << (1 * 2)) | (1U << (2 * 2)) | (1U << (3 * 2))); // output
    GPIOA->OTYPER &= ~((1U << 1) | (1U << 2) | (1U << 3));  // push-pull
    GPIOA->OSPEEDR |= ((3U << (1 * 2)) | (3U << (2 * 2)) | (3U << (3 * 2))); // very high speed
    GPIOA->PUPDR &= ~((3U << (1 * 2)) | (3U << (2 * 2)) | (3U << (3 * 2)));  // no pull
}

void ST7789_Fill(uint16_t color) {
    uint8_t hi = color >> 8, lo = color & 0xFF;

    // Set address window
    ST7789_Command(0x2A); // CASET
    ST7789_Data(0x00); ST7789_Data(0x00); // XSTART
    ST7789_Data(0x00); ST7789_Data(240-1); // XEND

    ST7789_Command(0x2B); // RASET
    ST7789_Data(0x00); ST7789_Data(0x00); // YSTART
    ST7789_Data(0x00); ST7789_Data(240-1); // YEND

    ST7789_Command(0x2C); // RAMWR

    DC_DATA();
    CS_LOW();
    for (uint32_t i = 0; i < 240 * 240; i++) {
        ST7789_Write(hi);
        ST7789_Write(lo);
    }
    CS_HIGH();
}
