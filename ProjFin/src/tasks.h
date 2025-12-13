#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "game.h"
#include "hx711.h"

// Estrutura para dados do sensor de carga
typedef struct {
    float weight;  // Força aplicada (0.0 a 1.0 normalizado)
} load_cell_data_t;

// Estrutura para relógio
typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} clock_time_t;

// Objetos de sincronização globais
extern QueueHandle_t load_cell_queue;
extern SemaphoreHandle_t render_semaphore;
extern SemaphoreHandle_t game_mutex;
extern SemaphoreHandle_t clock_mutex;
extern SemaphoreHandle_t display_mutex;

// Variáveis globais compartilhadas
extern game_state_t shared_game_state;
extern clock_time_t shared_clock;

// Protótipos das tarefas
void task_hx711(void *pvParameters);
void task_game_update(void *pvParameters);
void task_render(void *pvParameters);
void task_clock(void *pvParameters);
void task_buttons(void *pvParameters);

// Inicialização
void tasks_init(void);

