#pragma once
#include <stdint.h>

// Adaptação da engine do mini-golf-test para STM32F411 + ST7789
// Substitui as funções do raylib por funções do ST7789

#define KEY_COUNTER_CLOCKWISE 0
#define KEY_CLOCKWISE 1
#define KEY_SHOOT 2

typedef struct {
    float x;
    float y;
} Engine_vector2f;

typedef struct {
    int x;
    int y;
} Engine_vector2i;

typedef struct {
    float x;
    float y;
    float width;
    float height;
} Engine_rectangle;

// Inicialização
void engine_init_graphics(int width, int height);
void engine_deinit_graphics();
void engine_begin_drawing();
void engine_end_drawing();
float engine_get_delta_time();

// Entrada
int engine_is_button_pressed(int button);
float engine_get_shot_power(int screen_height);

// Renderização
void engine_clear_background();
void engine_draw_fps();
void engine_draw_circle(int center_x, int center_y, float radius);
void engine_draw_circle_lines(int center_x, int center_y, float radius);
void engine_draw_rectangle(int pos_x, int pos_y, int width, int height);
void engine_draw_rectangle_rect(Engine_rectangle rect);
void engine_draw_rectangle_rect_lines(Engine_rectangle rect);
void engine_draw_line(Engine_vector2f start_pos, Engine_vector2f end_pos);

