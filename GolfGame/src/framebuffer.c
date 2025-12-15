#include "framebuffer.h"
#include "font5x7.h"

// Implementação "C": API de framebuffer com backend em faixas ("stripes").
// Em vez de manter um framebuffer 240x240 completo em RAM (que não cabe),
// armazenamos apenas uma faixa horizontal pequena e uma lista de comandos
// de desenho. Em fb_present(), percorremos a tela por faixas, redesenhando
// todos os comandos apenas na parte visível daquela faixa e enviando por DMA.

// Altura da faixa em pixels (ajustar se quiser mais/menos RAM usada).
enum { FB_STRIPE_H = 120 };

// Buffer da faixa: FB_STRIPE_H linhas x LCD_W colunas.
static uint16_t fb_stripe[FB_STRIPE_H * LCD_W];

// Comandos suportados
typedef enum {
    FB_CMD_CLEAR,
    FB_CMD_FILL_RECT,
    FB_CMD_DRAW_RECT,
    FB_CMD_DRAW_LINE,
    FB_CMD_DRAW_CIRCLE,
    FB_CMD_FILL_CIRCLE,
    FB_CMD_TEXT_5x7
} fb_cmd_type_t;

typedef struct {
    fb_cmd_type_t type;
    union {
        struct { uint16_t color; } clear;
        struct { int x, y, w, h; uint16_t color; } fill_rect;
        struct { int x, y, w, h; uint16_t color; } draw_rect;
        struct { int x0, y0, x1, y1; uint16_t color; } line;
        struct { int x0, y0, r; uint16_t color; } circle;
        struct { int x0, y0, r; uint16_t color; } fill_circle;
        struct { int x, y; const char *s; uint16_t fg; int scale; int bg_en; uint16_t bg; } text;
    } params;
} fb_cmd_t;

// Número máximo de comandos por frame (ajuste se necessário).
enum { FB_MAX_CMDS = 64 };
static fb_cmd_t fb_cmds[FB_MAX_CMDS];
static int fb_cmd_count = 0;
static uint16_t fb_clear_color = 0x0000;

// ===================== Helpers internos de desenho =====================

static inline void stripe_put_pixel(int x, int y, int stripe_y0, uint16_t color) {
    if (x < 0 || x >= (int)LCD_W) return;
    if (y < stripe_y0 || y >= stripe_y0 + FB_STRIPE_H) return;
    int local_y = y - stripe_y0;
    fb_stripe[local_y * LCD_W + x] = color;
}

static void stripe_fill_rect(int x, int y, int w, int h, int stripe_y0, uint16_t color) {
    if (w <= 0 || h <= 0) return;
    int x0 = x;
    int y0 = y;
    int x1 = x + w;
    int y1 = y + h;
    if (x0 < 0) x0 = 0;
    if (x1 > (int)LCD_W) x1 = LCD_W;
    if (y1 <= stripe_y0 || y0 >= stripe_y0 + FB_STRIPE_H) return;
    if (y0 < stripe_y0) y0 = stripe_y0;
    if (y1 > stripe_y0 + FB_STRIPE_H) y1 = stripe_y0 + FB_STRIPE_H;
    for (int yy = y0; yy < y1; yy++) {
        int local_y = yy - stripe_y0;
        uint16_t *row = &fb_stripe[local_y * LCD_W];
        for (int xx = x0; xx < x1; xx++) {
            row[xx] = color;
        }
    }
}

static void stripe_draw_rect(int x, int y, int w, int h, int stripe_y0, uint16_t color) {
    if (w <= 0 || h <= 0) return;
    int x0 = x;
    int y0 = y;
    int x1 = x + w - 1;
    int y1 = y + h - 1;
    for (int xx = x0; xx <= x1; xx++) {
        stripe_put_pixel(xx, y0, stripe_y0, color);
        stripe_put_pixel(xx, y1, stripe_y0, color);
    }
    for (int yy = y0; yy <= y1; yy++) {
        stripe_put_pixel(x0, yy, stripe_y0, color);
        stripe_put_pixel(x1, yy, stripe_y0, color);
    }
}

static void stripe_draw_line(int x0, int y0, int x1, int y1, int stripe_y0, uint16_t color) {
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = (y1 > y0) ? (y0 - y1) : (y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        stripe_put_pixel(x0, y0, stripe_y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void stripe_draw_circle(int x0, int y0, int r, int stripe_y0, uint16_t color) {
    int x = r, y = 0, err = 0;
    while (x >= y) {
        stripe_put_pixel(x0 + x, y0 + y, stripe_y0, color);
        stripe_put_pixel(x0 + y, y0 + x, stripe_y0, color);
        stripe_put_pixel(x0 - y, y0 + x, stripe_y0, color);
        stripe_put_pixel(x0 - x, y0 + y, stripe_y0, color);
        stripe_put_pixel(x0 - x, y0 - y, stripe_y0, color);
        stripe_put_pixel(x0 - y, y0 - x, stripe_y0, color);
        stripe_put_pixel(x0 + y, y0 - x, stripe_y0, color);
        stripe_put_pixel(x0 + x, y0 - y, stripe_y0, color);
        y++;
        if (err <= 0) { err += 2 * y + 1; }
        if (err > 0)  { x--; err -= 2 * x + 1; }
    }
}

static void stripe_fill_circle(int x0, int y0, int r, int stripe_y0, uint16_t color) {
    int x = r, y = 0, err = 0;
    while (x >= y) {
        for (int xx = x0 - x; xx <= x0 + x; xx++) {
            stripe_put_pixel(xx, y0 + y, stripe_y0, color);
            stripe_put_pixel(xx, y0 - y, stripe_y0, color);
        }
        for (int xx = x0 - y; xx <= x0 + y; xx++) {
            stripe_put_pixel(xx, y0 + x, stripe_y0, color);
            stripe_put_pixel(xx, y0 - x, stripe_y0, color);
        }
        y++;
        if (err <= 0) { err += 2 * y + 1; }
        if (err > 0)  { x--; err -= 2 * x + 1; }
    }
}

static void stripe_draw_char_5x7(int x, int y, char ch, int stripe_y0,
                                 uint16_t fg, int scale, int bg_en, uint16_t bg) {
    if (ch < 32 || ch > 127) ch = '?';
    const uint8_t *col = FONT5x7[ch - 32];
    for (int cx = 0; cx < 5; cx++) {
        uint8_t bits = col[cx];
        for (int sx = 0; sx < scale; sx++) {
            int px = x + cx * scale + sx;
            if (px < 0 || px >= (int)LCD_W) continue;
            for (int cy = 0; cy < 7; cy++) {
                int py0 = y + cy * scale;
                uint16_t color = bg;
                int draw_bg = bg_en;
                if (bits & (1 << cy)) {
                    color = fg;
                    draw_bg = 1;
                }
                if (draw_bg) {
                    for (int sy = 0; sy < scale; sy++) {
                        int py = py0 + sy;
                        stripe_put_pixel(px, py, stripe_y0, color);
                    }
                }
            }
        }
    }
    if (bg_en) {
        for (int sx = 0; sx < scale; sx++) {
            int px = x + 5 * scale + sx;
            if (px < 0 || px >= (int)LCD_W) continue;
            for (int py = y; py < y + 7 * scale; py++) {
                stripe_put_pixel(px, py, stripe_y0, bg);
            }
        }
    }
}

static void stripe_draw_text_5x7(int x, int y, const char *s, int stripe_y0,
                                 uint16_t fg, int scale, int bg_en, uint16_t bg) {
    int cx = x, cy = y;
    if (scale < 1) scale = 1;
    while (*s) {
        if (*s == '\n') {
            cy += 8 * scale;
            cx = x;
            s++;
            continue;
        }
        stripe_draw_char_5x7(cx, cy, *s, stripe_y0, fg, scale, bg_en, bg);
        cx += 6 * scale;
        s++;
        if (cx >= (int)(LCD_W - 6 * scale)) {
            cy += 8 * scale;
            cx = x;
        }
        if (cy >= (int)(LCD_H - 8 * scale)) break;
    }
}

// ===================== Implementação da API pública =====================

void fb_clear(uint16_t color) {
    fb_clear_color = color;
    fb_cmd_count = 0; // nova frame
    if (fb_cmd_count < FB_MAX_CMDS) {
        fb_cmds[fb_cmd_count].type = FB_CMD_CLEAR;
        fb_cmds[fb_cmd_count].params.clear.color = color;
        fb_cmd_count++;
    }
}

void fb_fill_rect(int x, int y, int w, int h, uint16_t color) {
    if (fb_cmd_count >= FB_MAX_CMDS) return;
    fb_cmd_t *cmd = &fb_cmds[fb_cmd_count++];
    cmd->type = FB_CMD_FILL_RECT;
    cmd->params.fill_rect.x = x;
    cmd->params.fill_rect.y = y;
    cmd->params.fill_rect.w = w;
    cmd->params.fill_rect.h = h;
    cmd->params.fill_rect.color = color;
}

void fb_draw_rect(int x, int y, int w, int h, uint16_t color) {
    if (fb_cmd_count >= FB_MAX_CMDS) return;
    fb_cmd_t *cmd = &fb_cmds[fb_cmd_count++];
    cmd->type = FB_CMD_DRAW_RECT;
    cmd->params.draw_rect.x = x;
    cmd->params.draw_rect.y = y;
    cmd->params.draw_rect.w = w;
    cmd->params.draw_rect.h = h;
    cmd->params.draw_rect.color = color;
}

void fb_draw_line(int x0, int y0, int x1, int y1, uint16_t color) {
    if (fb_cmd_count >= FB_MAX_CMDS) return;
    fb_cmd_t *cmd = &fb_cmds[fb_cmd_count++];
    cmd->type = FB_CMD_DRAW_LINE;
    cmd->params.line.x0 = x0;
    cmd->params.line.y0 = y0;
    cmd->params.line.x1 = x1;
    cmd->params.line.y1 = y1;
    cmd->params.line.color = color;
}

void fb_draw_circle(int x0, int y0, int r, uint16_t color) {
    if (fb_cmd_count >= FB_MAX_CMDS) return;
    fb_cmd_t *cmd = &fb_cmds[fb_cmd_count++];
    cmd->type = FB_CMD_DRAW_CIRCLE;
    cmd->params.circle.x0 = x0;
    cmd->params.circle.y0 = y0;
    cmd->params.circle.r  = r;
    cmd->params.circle.color = color;
}

void fb_fill_circle(int x0, int y0, int r, uint16_t color) {
    if (fb_cmd_count >= FB_MAX_CMDS) return;
    fb_cmd_t *cmd = &fb_cmds[fb_cmd_count++];
    cmd->type = FB_CMD_FILL_CIRCLE;
    cmd->params.fill_circle.x0 = x0;
    cmd->params.fill_circle.y0 = y0;
    cmd->params.fill_circle.r  = r;
    cmd->params.fill_circle.color = color;
}

void fb_draw_text_5x7(int x, int y, const char *s, uint16_t fg, int scale, int bg_en, uint16_t bg) {
    if (fb_cmd_count >= FB_MAX_CMDS) return;
    fb_cmd_t *cmd = &fb_cmds[fb_cmd_count++];
    cmd->type = FB_CMD_TEXT_5x7;
    cmd->params.text.x = x;
    cmd->params.text.y = y;
    cmd->params.text.s = s;
    cmd->params.text.fg = fg;
    cmd->params.text.scale = scale;
    cmd->params.text.bg_en = bg_en;
    cmd->params.text.bg = bg;
}

void fb_present(void) {
    // Para cada faixa horizontal da tela:
    for (int stripe_y0 = 0; stripe_y0 < (int)LCD_H; stripe_y0 += FB_STRIPE_H) {
        int stripe_h = FB_STRIPE_H;
        if (stripe_y0 + stripe_h > (int)LCD_H) {
            stripe_h = (int)LCD_H - stripe_y0;
        }

        // 1) Preencher faixa com a cor de fundo
        for (int i = 0; i < stripe_h * (int)LCD_W; i++) {
            fb_stripe[i] = fb_clear_color;
        }

        // 2) Reproduzir todos os comandos nessa faixa
        for (int i = 0; i < fb_cmd_count; i++) {
            fb_cmd_t *cmd = &fb_cmds[i];
            switch (cmd->type) {
                case FB_CMD_CLEAR:
                    // já tratamos pela fb_clear_color
                    break;
                case FB_CMD_FILL_RECT:
                    stripe_fill_rect(cmd->params.fill_rect.x,
                                     cmd->params.fill_rect.y,
                                     cmd->params.fill_rect.w,
                                     cmd->params.fill_rect.h,
                                     stripe_y0,
                                     cmd->params.fill_rect.color);
                    break;
                case FB_CMD_DRAW_RECT:
                    stripe_draw_rect(cmd->params.draw_rect.x,
                                     cmd->params.draw_rect.y,
                                     cmd->params.draw_rect.w,
                                     cmd->params.draw_rect.h,
                                     stripe_y0,
                                     cmd->params.draw_rect.color);
                    break;
                case FB_CMD_DRAW_LINE:
                    stripe_draw_line(cmd->params.line.x0,
                                     cmd->params.line.y0,
                                     cmd->params.line.x1,
                                     cmd->params.line.y1,
                                     stripe_y0,
                                     cmd->params.line.color);
                    break;
                case FB_CMD_DRAW_CIRCLE:
                    stripe_draw_circle(cmd->params.circle.x0,
                                       cmd->params.circle.y0,
                                       cmd->params.circle.r,
                                       stripe_y0,
                                       cmd->params.circle.color);
                    break;
                case FB_CMD_FILL_CIRCLE:
                    stripe_fill_circle(cmd->params.fill_circle.x0,
                                       cmd->params.fill_circle.y0,
                                       cmd->params.fill_circle.r,
                                       stripe_y0,
                                       cmd->params.fill_circle.color);
                    break;
                case FB_CMD_TEXT_5x7:
                    stripe_draw_text_5x7(cmd->params.text.x,
                                         cmd->params.text.y,
                                         cmd->params.text.s,
                                         stripe_y0,
                                         cmd->params.text.fg,
                                         cmd->params.text.scale,
                                         cmd->params.text.bg_en,
                                         cmd->params.text.bg);
                    break;
                default:
                    break;
            }
        }

        // 3) Enviar a faixa atual para o LCD via DMA
        st7789_blit_region_dma(0, (uint16_t)stripe_y0, LCD_W, (uint16_t)stripe_h, fb_stripe);
    }
}

