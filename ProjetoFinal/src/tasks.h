#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "mpu6050.h"

// Estrutura para dados do acelerômetro
typedef struct {
    float ax, ay, az;  // Aceleração em g
    float gx, gy, gz;  // Giroscópio em dps
} accel_data_t;

// Estrutura para estado do jogo (compartilhado)
typedef struct {
    float ball_x, ball_y;
    float ball_vx, ball_vy;
    float aim_theta;  // Direção do tiro (radianos)
    float power;      // Potência do tiro (0.0 a 1.0)
    int shooting;     // 1 se a bola está em movimento
    int button_pressed; // 1 se botão foi pressionado
} game_state_t;

// Estrutura para relógio
typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} clock_time_t;

// Objetos de sincronização globais
extern QueueHandle_t accel_queue;
extern SemaphoreHandle_t render_semaphore;
extern SemaphoreHandle_t game_mutex;
extern SemaphoreHandle_t clock_mutex;

// Variáveis globais compartilhadas
extern game_state_t shared_game_state;
extern clock_time_t shared_clock;

// Protótipos das tarefas
void task_mpu6050(void *pvParameters);
void task_game_update(void *pvParameters);
void task_render(void *pvParameters);
void task_clock(void *pvParameters);
void task_button(void *pvParameters);

// Inicialização
void tasks_init(void);

