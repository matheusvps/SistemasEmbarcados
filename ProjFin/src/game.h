#pragma once
#include <stdint.h>
#include "board.h"

// Estrutura para estado do jogo
typedef struct {
    float ball_x, ball_y;        // Posição da bola
    float ball_vx, ball_vy;      // Velocidade da bola
    float aim_theta;             // Direção do tiro (radianos)
    float power;                 // Potência do tiro (0.0 a 1.0)
    int shooting;                // 1 se a bola está em movimento
    int strokes;                 // Número de tacadas
    int hole_x, hole_y;          // Posição do buraco
    int hole_radius;             // Raio do buraco
    int game_over;               // 1 se jogo terminou
} game_state_t;

// Funções do jogo
void game_init(game_state_t *state);
void game_update(game_state_t *state, float dt);
void game_reset(game_state_t *state);
int game_check_hole(game_state_t *state);

