#include "tasks.h"
#include "stm32f4xx.h"
#include "mpu6050.h"
#include "board.h"
#include "serial.h"
#include "st7789.h"
#include "framebuffer.h"
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

// Obstáculos por nível (até 5 níveis)
static const obstacle_t level1_obstacles[] = {
    { 110.0f, 60.0f, 20.0f, 70.0f },
    { 70.0f, 150.0f, 80.0f, 15.0f },
    { 160.0f, 170.0f, 15.0f, 40.0f },
};

static const obstacle_t level2_obstacles[] = {
    { 90.0f, 80.0f, 60.0f, 15.0f },
    { 40.0f, 130.0f, 20.0f, 60.0f },
    { 150.0f, 140.0f, 70.0f, 15.0f },
};

static const obstacle_t level3_obstacles[] = {
    { 60.0f, 60.0f, 120.0f, 15.0f },
    { 60.0f, 160.0f, 120.0f, 15.0f },
};

static const obstacle_t level4_obstacles[] = {
    { 80.0f, 60.0f, 20.0f, 80.0f },
    { 140.0f, 100.0f, 20.0f, 80.0f },
    { 40.0f, 150.0f, 160.0f, 15.0f },
};

static const obstacle_t level5_obstacles[] = {
    { 60.0f, 60.0f, 120.0f, 15.0f },
    { 60.0f, 180.0f, 120.0f, 15.0f },
    { 40.0f, 90.0f, 15.0f, 80.0f },
    { 185.0f, 90.0f, 15.0f, 30.0f },   // Parede direita superior (deixa abertura abaixo)
    { 185.0f, 140.0f, 15.0f, 30.0f },  // Parede direita inferior (deixa abertura no meio)
};

static void get_obstacles_for_level(uint8_t level,
                                    const obstacle_t** obstacles_out,
                                    size_t* count_out)
{
    switch (level) {
        case 1:
        default:
            *obstacles_out = level1_obstacles;
            *count_out = sizeof(level1_obstacles) / sizeof(level1_obstacles[0]);
            break;
        case 2:
            *obstacles_out = level2_obstacles;
            *count_out = sizeof(level2_obstacles) / sizeof(level2_obstacles[0]);
            break;
        case 3:
            *obstacles_out = level3_obstacles;
            *count_out = sizeof(level3_obstacles) / sizeof(level3_obstacles[0]);
            break;
        case 4:
            *obstacles_out = level4_obstacles;
            *count_out = sizeof(level4_obstacles) / sizeof(level4_obstacles[0]);
            break;
        case 5:
            *obstacles_out = level5_obstacles;
            *count_out = sizeof(level5_obstacles) / sizeof(level5_obstacles[0]);
            break;
    }
}

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
    // Habilitar clock do GPIOB (onde está o pushbutton PB7)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    BUTTON_SHOOT_PORT->MODER &= ~(3u << (BUTTON_SHOOT_PIN * 2));
    BUTTON_SHOOT_PORT->PUPDR &= ~(3u << (BUTTON_SHOOT_PIN * 2));
    BUTTON_SHOOT_PORT->PUPDR |= (1u << (BUTTON_SHOOT_PIN * 2)); // pull-up
}

// Colisão aproximada bola-retângulo usando AABB expandido.
// Mais simples de resolver (empurrar para fora por um dos quatro lados).
static int circle_rect_collision(float cx, float cy, float radius, const obstacle_t* ob) {
    // Verifica interseção entre o AABB da bola e o retângulo do obstáculo
    if (cx + radius < ob->x) return 0;
    if (cx - radius > ob->x + ob->w) return 0;
    if (cy + radius < ob->y) return 0;
    if (cy - radius > ob->y + ob->h) return 0;
    return 1;
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

    // Colisão com obstáculos retangulares (dependem do nível)
    const obstacle_t* obstacles;
    size_t obstacle_count;
    get_obstacles_for_level(state->level, &obstacles, &obstacle_count);

    for (size_t i = 0; i < obstacle_count; i++) {
        const obstacle_t* ob = &obstacles[i];
        if (!circle_rect_collision(state->ball_x, state->ball_y, BALL_RADIUS, ob)) {
            continue;
        }

        // Calcula a penetração em cada lado do obstáculo considerando o raio da bola.
        float left_pen   = (state->ball_x + BALL_RADIUS) - ob->x;                 // passou da esquerda
        float right_pen  = (ob->x + ob->w) - (state->ball_x - BALL_RADIUS);       // passou da direita
        float top_pen    = (state->ball_y + BALL_RADIUS) - ob->y;                 // passou do topo
        float bottom_pen = (ob->y + ob->h) - (state->ball_y - BALL_RADIUS);       // passou da base

        // Se qualquer penetração for negativa, não está realmente dentro
        if (left_pen <= 0.0f || right_pen <= 0.0f || top_pen <= 0.0f || bottom_pen <= 0.0f) {
            continue;
        }

        // Descobre o menor deslocamento necessário para sair do obstáculo
        float min_pen = left_pen;
        int side = 0; // 0 = esquerda, 1 = direita, 2 = topo, 3 = base

        if (right_pen < min_pen) { min_pen = right_pen; side = 1; }
        if (top_pen   < min_pen) { min_pen = top_pen;   side = 2; }
        if (bottom_pen < min_pen){ min_pen = bottom_pen;side = 3; }

        const float restitution = 0.6f;

        switch (side) {
            case 0: // Esquerda: empurrar bola para a esquerda do obstáculo
                state->ball_x = ob->x - BALL_RADIUS;
                state->ball_vx = -state->ball_vx * restitution;
                break;
            case 1: // Direita: empurrar bola para a direita do obstáculo
                state->ball_x = ob->x + ob->w + BALL_RADIUS;
                state->ball_vx = -state->ball_vx * restitution;
                break;
            case 2: // Topo: empurrar bola para cima
                state->ball_y = ob->y - BALL_RADIUS;
                state->ball_vy = -state->ball_vy * restitution;
                break;
            case 3: // Base: empurrar bola para baixo
                state->ball_y = ob->y + ob->h + BALL_RADIUS;
                state->ball_vy = -state->ball_vy * restitution;
                break;
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
    (void)pvParameters;
    const TickType_t xDelay = pdMS_TO_TICKS(40); // 25 Hz
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
    (void)pvParameters;
    const TickType_t xDelay = pdMS_TO_TICKS(50); // 60 FPS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    accel_data_t accel_data;
    // Passo de tempo deve bater com o período da tarefa (~50 ms)
    float dt = 0.05f; // 50ms
    int last_button_state = 0;
    // Variável para controlar a oscilação da barra de potência
    static float power_oscillation_time = 0.0f;
    const float POWER_OSCILLATION_SPEED = 2.0f; // Velocidade da oscilação (ciclos por segundo)
    
    printf("Tarefa Game Update iniciada\n");
    
    // Inicializar estado do jogo
    if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
        reset_ball(&shared_game_state);
        shared_game_state.level = 1;
        shared_game_state.score = 0;
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
            float angle = atan2f(accel_data.ax, accel_data.ay) + M_PI;
            
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
            // Atualizar oscilação da barra de potência (apenas quando não está atirando e não completou o buraco)
            if (!shared_game_state.shooting && !shared_game_state.hole_completed) {
                power_oscillation_time += dt * POWER_OSCILLATION_SPEED;
                // Usar seno para oscilar entre 0 e 1 (seno varia de -1 a 1, então ajustamos para 0 a 1)
                shared_game_state.power = (sinf(power_oscillation_time * 2.0f * M_PI) + 1.0f) * 0.5f;
            } else {
                // Resetar o tempo de oscilação quando começar a atirar ou completar o buraco
                power_oscillation_time = 0.0f;
            }
            
            // Permitir reset da fase após concluir o buraco
            if (shared_game_state.hole_completed && shared_game_state.button_pressed && !last_button_state) {
                shared_game_state.hole_completed = 0;
                shared_game_state.strokes = 0;
                // Avança de nível até o máximo de 5
                if (shared_game_state.level < 5) {
                    shared_game_state.level++;
                }
                reset_ball(&shared_game_state);
            }

            // Disparo quando botão físico é pressionado
            if (shared_game_state.button_pressed && !last_button_state && 
                !shared_game_state.shooting && !shared_game_state.hole_completed) {
                float shot_power = clampf(shared_game_state.power, 0.05f, 1.0f);
                // Aumenta o ganho de velocidade para deixar a bola mais rápida
                shared_game_state.ball_vx = shot_power * 320.0f * cosf(shared_game_state.aim_theta);
                shared_game_state.ball_vy = shot_power * 320.0f * sinf(shared_game_state.aim_theta);
                shared_game_state.shooting = 1;
                shared_game_state.power = 0.0f;
                shared_game_state.strokes++;
            }
            last_button_state = shared_game_state.button_pressed;
            
            // Atualizar posição da bola
            if (shared_game_state.shooting) {
                // Subdividir o passo de física em subpassos menores para reduzir tunneling
                const int substeps = 3;
                float sub_dt = dt / (float)substeps;

                for (int step = 0; step < substeps && shared_game_state.shooting; step++) {
                    shared_game_state.ball_x += shared_game_state.ball_vx * sub_dt;
                    shared_game_state.ball_y += shared_game_state.ball_vy * sub_dt;

                    // Fricção aplicada em cada subpasso
                    shared_game_state.ball_vx *= 0.97f;
                    shared_game_state.ball_vy *= 0.97f;

                    resolve_collisions(&shared_game_state);

                    // Parar se velocidade ficar abaixo de um limite (para o jogo ficar mais dinâmico)
                    float speed2 = shared_game_state.ball_vx * shared_game_state.ball_vx +
                                   shared_game_state.ball_vy * shared_game_state.ball_vy;
                    const float MIN_SPEED2 = 10.0f * 10.0f; // velocidade mínima ~10 px/s
                    if (speed2 < MIN_SPEED2) {
                        shared_game_state.ball_vx = 0.0f;
                        shared_game_state.ball_vy = 0.0f;
                        shared_game_state.shooting = 0;
                    }
                }
            }

            // Detectar vitória
            if (!shared_game_state.hole_completed && ball_in_hole(&shared_game_state)) {
                shared_game_state.hole_completed = 1;
                shared_game_state.shooting = 0;
                shared_game_state.ball_vx = 0.0f;
                shared_game_state.ball_vy = 0.0f;
                // Aumenta pontuação ao acertar o buraco
                shared_game_state.score += 1;
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

        // =================== Composição off-screen no framebuffer ===================
        // 1) Limpar framebuffer (fundo verde)
        fb_clear(C_GREEN); // Verde RGB565

        // 2) Desenhar obstáculos do nível atual
        const obstacle_t* render_obstacles;
        size_t render_obstacle_count;
        get_obstacles_for_level(state.level, &render_obstacles, &render_obstacle_count);
        for (size_t i = 0; i < render_obstacle_count; i++) {
            fb_fill_rect((int)render_obstacles[i].x, (int)render_obstacles[i].y,
                         (int)render_obstacles[i].w, (int)render_obstacles[i].h, 0x7BEF);
            fb_draw_rect((int)render_obstacles[i].x, (int)render_obstacles[i].y,
                         (int)render_obstacles[i].w, (int)render_obstacles[i].h, C_BLACK);
        }

        // 3) Desenhar buraco
        fb_fill_circle((int)HOLE_X, (int)HOLE_Y, HOLE_RADIUS + 4, C_BLACK);
        fb_fill_circle((int)HOLE_X, (int)HOLE_Y, HOLE_RADIUS, C_GREEN);

        // 4) Desenhar bola
        fb_fill_circle((int)state.ball_x, (int)state.ball_y, (int)BALL_RADIUS, C_WHITE);
        fb_draw_circle((int)state.ball_x, (int)state.ball_y, (int)BALL_RADIUS, C_BLACK);

        // 5) Desenhar linha de mira (se não estiver atirando)
        if (!state.shooting) {
            float end_x = state.ball_x + 40.0f * cosf(state.aim_theta);
            float end_y = state.ball_y + 40.0f * sinf(state.aim_theta);
            fb_draw_line((int)state.ball_x, (int)state.ball_y,
                         (int)end_x, (int)end_y, C_BLACK);
        }

        // 6) Desenhar barra de potência
        fb_draw_rect(10, 10, 10, 160, C_BLACK);
        int power_height = (int)(state.power * 160.0f);
        if (power_height > 0) {
            fb_fill_rect(11, 170 - power_height, 8, power_height, C_RED);
        }

        // 7) Status no canto inferior direito: apenas Nível X/5
        char info[24];
        snprintf(info, sizeof(info), "Nivel: %u/5", state.level);
        fb_draw_text_5x7(10, 228, info, C_WHITE, 1, 1, C_BLACK);

        if (state.hole_completed) {
            fb_draw_text_5x7(120, 210, "Buraco!", C_YELL, 2, 2, C_BLACK);
        }

        // 8) Desenhar relógio (hh:mm:ss)
        char time_str[12];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                 clock.hours, clock.minutes, clock.seconds);
        fb_draw_text_5x7(150, 5, time_str, C_WHITE, 1, 1, C_BLACK);

        // 9) Enviar framebuffer completo para o LCD em um único blit por DMA
        fb_present();
        
        // Temporização determinística
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// Tarefa de relógio
void task_clock(void *pvParameters) {
    (void)pvParameters;
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
    (void)pvParameters;
    const TickType_t xDelay = pdMS_TO_TICKS(50); // 20 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // Botão físico para disparo
    button_init();
    printf("Botão de disparo inicializado. Barra de potência oscila automaticamente.\n");
    
    for (;;) {
        // A barra de potência agora oscila automaticamente na tarefa task_game_update
        // Esta tarefa apenas lê o estado do botão físico
        if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
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

