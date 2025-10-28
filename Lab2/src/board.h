#pragma once
#include "stm32f4xx.h"

/* Dimensões do painel */
#define LCD_W 240
#define LCD_H 240

/* Pinos (mapeamento que funcionou) */
#define LCD_DC_PORT   GPIOB
#define LCD_DC_PIN    0
#define LCD_RST_PORT  GPIOB
#define LCD_RST_PIN   1
#define LCD_BLK_PORT  GPIOB
#define LCD_BLK_PIN   6
#define LCD_CS_PORT   GPIOB  /* opcional; manter LOW evita flutuação */
#define LCD_CS_PIN    10

/* SPI1: SCK=PA5, MOSI=PA7 (MISO não usado) */

/* Cores 16-bit RGB565 úteis */
#define C_BLACK 0x0000
#define C_WHITE 0xFFFF
#define C_RED   0xF800
#define C_GREEN 0x07E0
#define C_BLUE  0x001F
#define C_YELL  0xFFE0
#define C_CYAN  0x07FF
#define C_MAG   0xF81F
