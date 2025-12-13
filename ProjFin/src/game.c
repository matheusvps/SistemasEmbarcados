#include "game.h"
#include <math.h>

#define FRICTION 0.98f
#define MIN_VELOCITY 0.5f
#define BALL_RADIUS 5
#define HOLE_RADIUS 8

void game_init(game_state_t *state) {
    state->ball_x = 50.0f;
    state->ball_y = 120.0f;
    state->ball_vx = 0.0f;
    state->ball_vy = 0.0f;
    state->aim_theta = 0.0f;
    state->power = 0.0f;
    state->shooting = 0;
    state->strokes = 0;
    state->hole_x = 200;
    state->hole_y = 120;
    state->hole_radius = HOLE_RADIUS;
    state->game_over = 0;
}

void game_reset(game_state_t *state) {
    game_init(state);
}

int game_check_hole(game_state_t *state) {
    float dx = state->ball_x - state->hole_x;
    float dy = state->ball_y - state->hole_y;
    float dist = sqrtf(dx*dx + dy*dy);
    return (dist < (BALL_RADIUS + state->hole_radius)) && 
           (fabsf(state->ball_vx) < 2.0f) && 
           (fabsf(state->ball_vy) < 2.0f);
}

void game_update(game_state_t *state, float dt) {
    if (state->game_over) return;
    
    // Atualizar posição da bola se estiver em movimento
    if (state->shooting) {
        state->ball_x += state->ball_vx * dt;
        state->ball_y += state->ball_vy * dt;
        
        // Aplicar fricção
        state->ball_vx *= FRICTION;
        state->ball_vy *= FRICTION;
        
        // Parar se velocidade muito baixa
        float speed = sqrtf(state->ball_vx*state->ball_vx + state->ball_vy*state->ball_vy);
        if (speed < MIN_VELOCITY) {
            state->ball_vx = 0.0f;
            state->ball_vy = 0.0f;
            state->shooting = 0;
        }
        
        // Colisões com bordas
        if (state->ball_x - BALL_RADIUS < 0) {
            state->ball_x = BALL_RADIUS;
            state->ball_vx = -state->ball_vx * 0.8f;
        }
        if (state->ball_x + BALL_RADIUS > LCD_W) {
            state->ball_x = LCD_W - BALL_RADIUS;
            state->ball_vx = -state->ball_vx * 0.8f;
        }
        if (state->ball_y - BALL_RADIUS < 30) {  // Considerar cabeçalho
            state->ball_y = 30 + BALL_RADIUS;
            state->ball_vy = -state->ball_vy * 0.8f;
        }
        if (state->ball_y + BALL_RADIUS > LCD_H) {
            state->ball_y = LCD_H - BALL_RADIUS;
            state->ball_vy = -state->ball_vy * 0.8f;
        }
        
        // Verificar se caiu no buraco
        if (game_check_hole(state)) {
            state->shooting = 0;
            state->game_over = 1;
        }
    }
}

