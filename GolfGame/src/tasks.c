#include "tasks.h"
#include "stm32f4xx.h"
#include "mpu6050.h"
#include "board.h"
#include "serial.h"
#include "st7789.h"
#include "hx711.h"
#include <stdio.h>
#include <math.h>
#include <stddef.h>

#define FIELD_MIN_X 10.0f
#define FIELD_MAX_X 230.0f
#define FIELD_MIN_Y 20.0f
#define FIELD_MAX_Y 220.0f
#define BALL_RADIUS 8.0f
#define HOLE_X 200.0f
#define HOLE_Y 120.0f
#define HOLE_RADIUS 12.0f
#define MAX_FORCE_KG 1.0f

typedef struct {
    float x;
    float y;
    float w;
    float h;
} obstacle_t;

static const obstacle_t obstacles[] = {
    { 110.0f, 60.0f, 20.0f, 70.0f },
    { 70.0f, 150.0f, 80.0f, 15.0f },
    { 160.0f, 170.0f, 15.0f, 40.0f },
};
static const size_t obstacle_count = sizeof(obstacles) / sizeof(obstacles[0]);

// Objetos de sincronização
QueueHandle_t accel_queue;
SemaphoreHandle_t game_mutex;
SemaphoreHandle_t clock_mutex;

// Variáveis compartilhadas
game_state_t shared_game_state;
clock_time_t shared_clock;

// Máquina de estados para MPU6050
typedef enum {
    MPU_STATE_INIT,
    MPU_STATE_WAIT,
    MPU_STATE_READ,
    MPU_STATE_ERROR
} mpu_state_t;

static mpu_state_t mpu_state = MPU_STATE_INIT;
static uint32_t mpu_error_count = 0;

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float sqr(float v) { return v * v; }

static inline int button_is_pressed(void) {
    // Botão com pull-up interno: pressionado = nível lógico baixo
    return (BUTTON_SHOOT_PORT->IDR & (1u << BUTTON_SHOOT_PIN)) == 0;
}

static void button_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    BUTTON_SHOOT_PORT->MODER &= ~(3u << (BUTTON_SHOOT_PIN * 2));
    BUTTON_SHOOT_PORT->PUPDR &= ~(3u << (BUTTON_SHOOT_PIN * 2));
    BUTTON_SHOOT_PORT->PUPDR |= (1u << (BUTTON_SHOOT_PIN * 2)); // pull-up
}

static int circle_rect_collision(float cx, float cy, float radius, const obstacle_t* ob) {
    float closest_x = clampf(cx, ob->x, ob->x + ob->w);
    float closest_y = clampf(cy, ob->y, ob->y + ob->h);
    float dx = cx - closest_x;
    float dy = cy - closest_y;
    return (dx * dx + dy * dy) < (radius * radius);
}

static void resolve_collisions(game_state_t* state) {
    // Colisão com paredes
    if (state->ball_x - BALL_RADIUS < FIELD_MIN_X) {
        state->ball_x = FIELD_MIN_X + BALL_RADIUS;
        state->ball_vx = -state->ball_vx * 0.6f;
    } else if (state->ball_x + BALL_RADIUS > FIELD_MAX_X) {
        state->ball_x = FIELD_MAX_X - BALL_RADIUS;
        state->ball_vx = -state->ball_vx * 0.6f;
    }

    if (state->ball_y - BALL_RADIUS < FIELD_MIN_Y) {
        state->ball_y = FIELD_MIN_Y + BALL_RADIUS;
        state->ball_vy = -state->ball_vy * 0.6f;
    } else if (state->ball_y + BALL_RADIUS > FIELD_MAX_Y) {
        state->ball_y = FIELD_MAX_Y - BALL_RADIUS;
        state->ball_vy = -state->ball_vy * 0.6f;
    }

    // Colisão com obstáculos retangulares
    for (size_t i = 0; i < obstacle_count; i++) {
        if (circle_rect_collision(state->ball_x, state->ball_y, BALL_RADIUS, &obstacles[i])) {
            // Empurrar bola para fora do obstáculo invertendo componente dominante
            float center_x = obstacles[i].x + obstacles[i].w / 2.0f;
            float center_y = obstacles[i].y + obstacles[i].h / 2.0f;
            float dx = state->ball_x - center_x;
            float dy = state->ball_y - center_y;
            if (fabsf(dx) > fabsf(dy)) {
                state->ball_vx = -state->ball_vx * 0.6f;
                state->ball_x += (dx > 0 ? 2.0f : -2.0f);
            } else {
                state->ball_vy = -state->ball_vy * 0.6f;
                state->ball_y += (dy > 0 ? 2.0f : -2.0f);
            }
        }
    }
}

static int ball_in_hole(const game_state_t* state) {
    float dx = state->ball_x - HOLE_X;
    float dy = state->ball_y - HOLE_Y;
    return (sqr(dx) + sqr(dy)) <= sqr(HOLE_RADIUS - 2.0f);
}

static void reset_ball(game_state_t* state) {
    state->ball_x = 50.0f;
    state->ball_y = 120.0f;
    state->ball_vx = 0.0f;
    state->ball_vy = 0.0f;
    state->power = 0.0f;
    state->shooting = 0;
}

// Tarefa MPU6050 com máquina de estados
void task_mpu6050(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(20); // 50 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    mpu6050_raw_t raw_data;
    accel_data_t accel_data;
    
    printf("Tarefa MPU6050 iniciada\n");
    
    for (;;) {
        // Máquina de estados
        switch (mpu_state) {
            case MPU_STATE_INIT:
                // Tentar inicializar
                if (mpu6050_init() == 0) {
                    mpu_state = MPU_STATE_READ;
                    printf("MPU6050 inicializado\n");
                } else {
                    mpu_state = MPU_STATE_ERROR;
                    mpu_error_count++;
                }
                break;
                
            case MPU_STATE_WAIT:
                // Aguardar antes de tentar novamente
                vTaskDelay(pdMS_TO_TICKS(1000));
                mpu_state = MPU_STATE_INIT;
                break;
                
            case MPU_STATE_READ:
                // Ler dados do sensor
                if (mpu6050_read_all(&raw_data) == 0) {
                    // Converter para unidades físicas
                    accel_data.ax = mpu6050_accel_g(raw_data.ax);
                    accel_data.ay = mpu6050_accel_g(raw_data.ay);
                    accel_data.az = mpu6050_accel_g(raw_data.az);
                    accel_data.gx = mpu6050_gyro_dps(raw_data.gx);
                    accel_data.gy = mpu6050_gyro_dps(raw_data.gy);
                    accel_data.gz = mpu6050_gyro_dps(raw_data.gz);
                    
                    // Enviar para fila (não bloqueante)
                    if (xQueueSend(accel_queue, &accel_data, 0) != pdTRUE) {
                        // Fila cheia, descartar dado antigo
                        accel_data_t dummy;
                        xQueueReceive(accel_queue, &dummy, 0);
                        xQueueSend(accel_queue, &accel_data, 0);
                    }
                    mpu_error_count = 0;
                } else {
                    mpu_error_count++;
                    if (mpu_error_count > 10) {
                        mpu_state = MPU_STATE_ERROR;
                    }
                }
                break;
                
            case MPU_STATE_ERROR:
                printf("Erro MPU6050, tentando reinicializar...\n");
                mpu_error_count = 0;
                mpu_state = MPU_STATE_WAIT;
                break;
        }
        
        // Temporização determinística
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// Tarefa de atualização do jogo (física + renderização)
void task_game_update(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(16); // ~60 FPS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    accel_data_t accel_data;
    float dt = 0.016f; // 16ms
    int last_button_state = 0;
    
    printf("Tarefa Game Update iniciada\n");
    
    // Inicializar estado do jogo
    if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
        reset_ball(&shared_game_state);
        shared_game_state.aim_theta = 0.0f;
        shared_game_state.button_pressed = 0;
        shared_game_state.strokes = 0;
        shared_game_state.hole_completed = 0;
        xSemaphoreGive(game_mutex);
    }
    
    for (;;) {
        // Ler dados do acelerômetro da fila
        if (xQueueReceive(accel_queue, &accel_data, 0) == pdTRUE) {
            // Usar acelerômetro para definir direção
            // ax e ay definem a direção do tiro
            float angle = atan2f(accel_data.ay, accel_data.ax);
            
            if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
                if (!shared_game_state.shooting) {
                    // Atualizar direção apenas se não estiver atirando
                    shared_game_state.aim_theta = angle;
                }
                xSemaphoreGive(game_mutex);
            }
        }
        
        // Atualizar física do jogo
        if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
            // Permitir reset da fase após concluir o buraco
            if (shared_game_state.hole_completed && shared_game_state.button_pressed && !last_button_state) {
                shared_game_state.hole_completed = 0;
                shared_game_state.strokes = 0;
                reset_ball(&shared_game_state);
            }

            // Disparo quando botão físico é pressionado
            if (shared_game_state.button_pressed && !last_button_state && 
                !shared_game_state.shooting && !shared_game_state.hole_completed) {
                float shot_power = clampf(shared_game_state.power, 0.05f, 1.0f);
                shared_game_state.ball_vx = shot_power * 220.0f * cosf(shared_game_state.aim_theta);
                shared_game_state.ball_vy = shot_power * 220.0f * sinf(shared_game_state.aim_theta);
                shared_game_state.shooting = 1;
                shared_game_state.power = 0.0f;
                shared_game_state.strokes++;
            }
            last_button_state = shared_game_state.button_pressed;
            
            // Atualizar posição da bola
            if (shared_game_state.shooting) {
                shared_game_state.ball_x += shared_game_state.ball_vx * dt;
                shared_game_state.ball_y += shared_game_state.ball_vy * dt;
                
                // Fricção
                shared_game_state.ball_vx *= 0.95f;
                shared_game_state.ball_vy *= 0.95f;
                
                resolve_collisions(&shared_game_state);

                // Parar se velocidade muito baixa
                if (fabsf(shared_game_state.ball_vx) < 1.0f && 
                    fabsf(shared_game_state.ball_vy) < 1.0f) {
                    shared_game_state.ball_vx = 0.0f;
                    shared_game_state.ball_vy = 0.0f;
                    shared_game_state.shooting = 0;
                }
            }

            // Detectar vitória
            if (!shared_game_state.hole_completed && ball_in_hole(&shared_game_state)) {
                shared_game_state.hole_completed = 1;
                shared_game_state.shooting = 0;
                shared_game_state.ball_vx = 0.0f;
                shared_game_state.ball_vy = 0.0f;
            }
            
            xSemaphoreGive(game_mutex);
        }

        // --- Renderização (usa um snapshot do estado compartilhado) ---
        game_state_t state;
        if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
            state = shared_game_state;
            xSemaphoreGive(game_mutex);
        } else {
            // Se não conseguiu pegar o mutex, pula o frame
            vTaskDelayUntil(&xLastWakeTime, xDelay);
            continue;
        }

        clock_time_t clock;
        if (xSemaphoreTake(clock_mutex, 0) == pdTRUE) {
            clock = shared_clock;
            xSemaphoreGive(clock_mutex);
        } else {
            clock.hours = clock.minutes = clock.seconds = 0;
        }

        // Limpar tela (fundo verde)
        st7789_fill_screen(0x4354); // Verde RGB565

        // Desenhar obstáculos
        for (size_t i = 0; i < obstacle_count; i++) {
            st7789_fill_rect((uint16_t)obstacles[i].x, (uint16_t)obstacles[i].y,
                             (uint16_t)obstacles[i].w, (uint16_t)obstacles[i].h, 0x7BEF);
            st7789_draw_rect((uint16_t)obstacles[i].x, (uint16_t)obstacles[i].y,
                             (uint16_t)obstacles[i].w, (uint16_t)obstacles[i].h, C_BLACK);
        }

        // Desenhar buraco
        st7789_fill_circle((int)HOLE_X, (int)HOLE_Y, HOLE_RADIUS + 4, C_BLACK);
        st7789_fill_circle((int)HOLE_X, (int)HOLE_Y, HOLE_RADIUS, C_GREEN);

        // Desenhar bola
        st7789_fill_circle((int)state.ball_x, (int)state.ball_y, (int)BALL_RADIUS, C_WHITE);
        st7789_draw_circle((int)state.ball_x, (int)state.ball_y, (int)BALL_RADIUS, C_BLACK);

        // Desenhar linha de mira (se não estiver atirando)
        if (!state.shooting) {
            float end_x = state.ball_x + 40.0f * cosf(state.aim_theta);
            float end_y = state.ball_y + 40.0f * sinf(state.aim_theta);
            st7789_draw_line((int)state.ball_x, (int)state.ball_y,
                             (int)end_x, (int)end_y, C_BLACK);
        }

        // Desenhar barra de potência
        st7789_draw_rect(10, 10, 10, 160, C_BLACK);
        int power_height = (int)(state.power * 160.0f);
        st7789_fill_rect(11, 170 - power_height, 8, power_height, C_RED);

        // Pontuação e status
        char info[24];
        snprintf(info, sizeof(info), "Tacadas: %u", state.strokes);
        st7789_draw_text_5x7(25, 190, info, C_WHITE, 1, 1, C_BLACK);

        if (state.hole_completed) {
            st7789_draw_text_5x7(120, 210, "Buraco!", C_YELL, 2, 2, C_BLACK);
        }

        // Desenhar relógio (hh:mm:ss)
        char time_str[9];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                 clock.hours, clock.minutes, clock.seconds);
        st7789_draw_text_5x7(150, 5, time_str, C_WHITE, 1, 1, C_BLACK);
        
        // Temporização determinística
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// Tarefa de relógio
void task_clock(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000); // 1 segundo
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    printf("Tarefa Clock iniciada\n");
    
    // Inicializar relógio
    if (xSemaphoreTake(clock_mutex, portMAX_DELAY) == pdTRUE) {
        shared_clock.hours = 0;
        shared_clock.minutes = 0;
        shared_clock.seconds = 0;
        xSemaphoreGive(clock_mutex);
    }
    
    for (;;) {
        // Atualizar relógio a cada segundo
        if (xSemaphoreTake(clock_mutex, portMAX_DELAY) == pdTRUE) {
            shared_clock.seconds++;
            if (shared_clock.seconds >= 60) {
                shared_clock.seconds = 0;
                shared_clock.minutes++;
                if (shared_clock.minutes >= 60) {
                    shared_clock.minutes = 0;
                    shared_clock.hours++;
                    if (shared_clock.hours >= 24) {
                        shared_clock.hours = 0;
                    }
                }
            }
            xSemaphoreGive(clock_mutex);
        }
        
        // Temporização determinística
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// Tarefa de leitura da célula de carga (substitui botão)
void task_button(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(50); // 20 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // Inicializar HX711
    printf("Inicializando HX711 (Célula de Carga)...\n");
    hx711_init();
    printf("HX711 inicializado. Fazendo tara...\n");
    
    // Botão físico para disparo
    button_init();

    // Aguardar um pouco para estabilizar
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Fazer tara (zerar)
    hx711_tare();
    printf("Tara concluída. Célula de carga pronta.\n");
    
    for (;;) {
        // Ler força da célula de carga
        float force = hx711_read_weight();  // Força em kg
        const float MIN_FORCE = 0.02f;  // 20g mínimo para reduzir ruído
        float normalized = 0.0f;
        if (force > MIN_FORCE) {
            normalized = clampf(force / MAX_FORCE_KG, 0.0f, 1.0f);
        }
        
        if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
            shared_game_state.power = normalized;
            shared_game_state.button_pressed = button_is_pressed();
            xSemaphoreGive(game_mutex);
        }
        
        // Temporização determinística
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// Inicialização das tarefas
void tasks_init(void) {
    // Criar objetos de sincronização
    accel_queue = xQueueCreate(5, sizeof(accel_data_t));
    game_mutex = xSemaphoreCreateMutex();
    clock_mutex = xSemaphoreCreateMutex();
    
    // Criar tarefas
    xTaskCreate(task_mpu6050, "MPU6050", 256, NULL, 3, NULL);
    xTaskCreate(task_game_update, "GameUpdate", 512, NULL, 2, NULL);
    xTaskCreate(task_clock, "Clock", 128, NULL, 2, NULL);
    xTaskCreate(task_button, "LoadCell", 256, NULL, 2, NULL);  // Aumentado stack para HX711
    
    printf("Tarefas criadas\n");
}

