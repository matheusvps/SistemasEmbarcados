    #ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <stdint.h>
#include "st7789.h"  // para LCD_W / LCD_H e tipos de cor RGB565

// API de desenho em framebuffer off-screen (RGB565, resolução igual ao LCD).
// A implementação mantém um framebuffer estático em RAM e, ao final do frame,
// envia tudo de uma vez para o LCD usando DMA.

// Limpa o framebuffer inteiro com uma cor sólida.
void fb_clear(uint16_t color);

// Primitivas básicas
void fb_fill_rect(int x, int y, int w, int h, uint16_t color);
void fb_draw_rect(int x, int y, int w, int h, uint16_t color);
void fb_draw_line(int x0, int y0, int x1, int y1, uint16_t color);
void fb_draw_circle(int x0, int y0, int r, uint16_t color);
void fb_fill_circle(int x0, int y0, int r, uint16_t color);

// Texto 5x7 (mesma fonte usada pelo driver do LCD)
void fb_draw_text_5x7(int x, int y, const char *s, uint16_t fg, int scale, int bg_en, uint16_t bg);

// Envia o framebuffer atual para o LCD (full screen) usando DMA.
void fb_present(void);

#endif // FRAMEBUFFER_H


