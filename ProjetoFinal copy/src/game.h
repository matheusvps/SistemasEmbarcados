#pragma once
#include "engine.h"

// Lógica do jogo Mini-Golf
// Adaptada de mini-golf-test/main.c

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240

#define BALL_RADIUS 8.0
#define TRACER_LENGTH 40.0
#define TRACER_BALL_DISTANCE (2 * BALL_RADIUS)
#define HOLE_RADIUS 20.0
#define POWER_BAR_HEIGHT (4 * SCREEN_HEIGHT / 6.0)
#define POWER_BAR_WIDTH  (SCREEN_HEIGHT / 12.0)

typedef struct {
    Engine_vector2f pos;
    Engine_vector2f size;
} Entity;

typedef struct {
    Engine_vector2f pos;
    Engine_vector2f vel;
    float   radius;
    Engine_vector2f aim_direction;
    float   aim_theta;
    float   power;
} Ball;

typedef struct {
    Engine_vector2f start_pos;
    Engine_vector2f end_pos;
} Tracer;

typedef struct {
    float dt;
    Ball ball;
    Tracer tracer;
    Engine_rectangle power_bar;
    Entity hole;
    Entity* obstacles;
    int obstacle_count;
} Game_state;

// Funções do jogo
int ball_is_shooting(Game_state* state);
void init_game_state(Game_state* state);
void shoot(Game_state* state);
void update_state(Game_state* state);
void render(Game_state* state);

