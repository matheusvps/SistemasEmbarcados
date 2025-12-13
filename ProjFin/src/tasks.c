#include "tasks.h"
#include "stm32f4xx.h"
#include "hx711.h"
#include "board.h"
#include "serial.h"
#include "st7789.h"
#include "game.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Objetos de sincronização
QueueHandle_t load_cell_queue;
SemaphoreHandle_t render_semaphore;
SemaphoreHandle_t game_mutex;
SemaphoreHandle_t clock_mutex;
SemaphoreHandle_t display_mutex;

// Variáveis compartilhadas
game_state_t shared_game_state;
clock_time_t shared_clock;

// Máquina de estados para HX711
typedef enum {
    HX711_STATE_INIT,
    HX711_STATE_WAIT,
    HX711_STATE_READ,
    HX711_STATE_ERROR
} hx711_state_t;

static hx711_state_t hx711_state = HX711_STATE_INIT;
static uint32_t hx711_error_count = 0;

// Tarefa HX711 com máquina de estados
void task_hx711(void *pvParameters) {
    (void)pvParameters; // Suprimir warning
    const TickType_t xDelay = pdMS_TO_TICKS(20); // 50 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    load_cell_data_t load_data;
    
    printf("Tarefa HX711 iniciada\n");
    
    for (;;) {
        // Máquina de estados
        switch (hx711_state) {
            case HX711_STATE_INIT: {
                // Tentar inicializar
                hx711_init();
                hx711_state = HX711_STATE_READ;
                printf("HX711 inicializado\n");
                break;
            }
                
            case HX711_STATE_WAIT: {
                // Aguardar antes de tentar novamente
                vTaskDelay(pdMS_TO_TICKS(1000));
                hx711_state = HX711_STATE_INIT;
                break;
            }
                
            case HX711_STATE_READ: {
                // Ler dados do sensor
                float weight = hx711_read_weight();
                if (weight >= 0.0f) {
                    // Normalizar para 0.0 a 1.0 (ajustar conforme necessário)
                    load_data.weight = weight;
                    if (load_data.weight > 1.0f) load_data.weight = 1.0f;
                    
                    // Enviar para fila (não bloqueante)
                    if (xQueueSend(load_cell_queue, &load_data, 0) != pdTRUE) {
                        // Fila cheia, descartar dado antigo
                        load_cell_data_t dummy;
                        xQueueReceive(load_cell_queue, &dummy, 0);
                        xQueueSend(load_cell_queue, &load_data, 0);
                    }
                    hx711_error_count = 0;
                } else {
                    hx711_error_count++;
                    if (hx711_error_count > 10) {
                        hx711_state = HX711_STATE_ERROR;
                    }
                }
                break;
            }
                
            case HX711_STATE_ERROR: {
                printf("Erro HX711, tentando reinicializar...\n");
                hx711_error_count = 0;
                hx711_state = HX711_STATE_WAIT;
                break;
            }
        }
        
        // Temporização determinística
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// Tarefa de atualização do jogo
void task_game_update(void *pvParameters) {
    (void)pvParameters; // Suprimir warning
    const TickType_t xDelay = pdMS_TO_TICKS(16); // ~60 FPS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    load_cell_data_t load_data;
    float dt = 0.016f; // 16ms
    
    printf("Tarefa Game Update iniciada\n");
    
    // Inicializar estado do jogo
    if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
        game_init(&shared_game_state);
        xSemaphoreGive(game_mutex);
    }
    
    for (;;) {
        // Ler dados do sensor de carga da fila
        if (xQueueReceive(load_cell_queue, &load_data, 0) == pdTRUE) {
            if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
                if (!shared_game_state.shooting) {
                    // Atualizar potência baseada no sensor
                    shared_game_state.power = load_data.weight;
                }
                xSemaphoreGive(game_mutex);
            }
        }
        
        // Atualizar física do jogo
        if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
            game_update(&shared_game_state, dt);
            xSemaphoreGive(game_mutex);
        }
        
        // Sinalizar renderização
        xSemaphoreGive(render_semaphore);
        
        // Temporização determinística
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// Tarefa de renderização
void task_render(void *pvParameters) {
    (void)pvParameters; // Suprimir warning
    const TickType_t xDelay = pdMS_TO_TICKS(33); // ~30 FPS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    game_state_t local_state = {0}; // Inicializar com zeros
    clock_time_t local_clock = {0}; // Inicializar com zeros
    char time_str[12]; // Aumentado para 12 bytes
    char strokes_str[20];
    
    printf("Tarefa Render iniciada\n");
    
    for (;;) {
        // Aguardar semáforo de renderização
        if (xSemaphoreTake(render_semaphore, portMAX_DELAY) == pdTRUE) {
            // Copiar estado do jogo
            if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
                local_state = shared_game_state;
                xSemaphoreGive(game_mutex);
            }
            
            // Copiar relógio
            if (xSemaphoreTake(clock_mutex, portMAX_DELAY) == pdTRUE) {
                local_clock = shared_clock;
                xSemaphoreGive(clock_mutex);
            }
            
            // Proteger acesso ao display
            if (xSemaphoreTake(display_mutex, portMAX_DELAY) == pdTRUE) {
                // Limpar tela
                st7789_fill_screen(C_BLACK);
                
                // Desenhar cabeçalho com relógio
                st7789_fill_rect(0, 0, LCD_W, 30, C_BLACK);
                sprintf(time_str, "%02d:%02d:%02d", 
                        local_clock.hours, local_clock.minutes, local_clock.seconds);
                st7789_draw_text_5x7(LCD_W - 80, 5, time_str, C_WHITE, 1, 0, 0);
                
                // Desenhar pontuação
                sprintf(strokes_str, "Strokes: %d", local_state.strokes);
                st7789_draw_text_5x7(5, 5, strokes_str, C_WHITE, 1, 0, 0);
                
                // Desenhar buraco
                st7789_fill_circle(local_state.hole_x, local_state.hole_y, 
                                  local_state.hole_radius, C_BLACK);
                st7789_draw_circle(local_state.hole_x, local_state.hole_y, 
                                  local_state.hole_radius, C_WHITE);
                
                // Desenhar bola
                st7789_fill_circle((int)local_state.ball_x, (int)local_state.ball_y, 
                                  5, C_WHITE);
                
                // Desenhar linha de mira se não estiver atirando
                if (!local_state.shooting) {
                    float aim_len = 30.0f;
                    int x1 = (int)local_state.ball_x;
                    int y1 = (int)local_state.ball_y;
                    int x2 = (int)(local_state.ball_x + aim_len * cosf(local_state.aim_theta));
                    int y2 = (int)(local_state.ball_y + aim_len * sinf(local_state.aim_theta));
                    st7789_draw_line(x1, y1, x2, y2, C_CYAN);
                }
                
                // Desenhar barra de potência
                if (!local_state.shooting) {
                    int bar_width = (int)(local_state.power * 100.0f);
                    st7789_fill_rect(5, LCD_H - 25, bar_width, 15, C_RED);
                    st7789_draw_rect(5, LCD_H - 25, 100, 15, C_WHITE);
                }
                
                // Mensagem de fim de jogo
                if (local_state.game_over) {
                    st7789_draw_text_5x7(70, 100, "HOLE IN ONE!", C_GREEN, 2, 1, C_BLACK);
                }
                
                xSemaphoreGive(display_mutex);
            }
        }
        
        // Temporização determinística
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// Tarefa de relógio
void task_clock(void *pvParameters) {
    (void)pvParameters; // Suprimir warning
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
        // Atualizar relógio
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

// Tarefa de botões
void task_buttons(void *pvParameters) {
    (void)pvParameters; // Suprimir warning
    const TickType_t xDelay = pdMS_TO_TICKS(50); // 20 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static float last_power = 0.0f;
    static float max_power = 0.0f;
    
    // Inicializar botões
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    // Configurar botões como entrada com pull-up
    GPIOA->MODER &= ~((3u << (BTN_CW_PIN * 2)) | (3u << (BTN_CCW_PIN * 2)));
    GPIOA->PUPDR &= ~((3u << (BTN_CW_PIN * 2)) | (3u << (BTN_CCW_PIN * 2)));
    GPIOA->PUPDR |= ((1u << (BTN_CW_PIN * 2)) | (1u << (BTN_CCW_PIN * 2)));
    
    printf("Tarefa Buttons iniciada\n");
    
    for (;;) {
        // Ler estado dos botões
        int btn_cw = !(GPIOA->IDR & (1u << BTN_CW_PIN));   // LOW quando pressionado
        int btn_ccw = !(GPIOA->IDR & (1u << BTN_CCW_PIN)); // LOW quando pressionado
        
        if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
            if (!shared_game_state.shooting) {
                // Atualizar direção com botões
                if (btn_cw && !btn_ccw) {
                    shared_game_state.aim_theta += 0.05f; // Sentido horário
                }
                if (btn_ccw && !btn_cw) {
                    shared_game_state.aim_theta -= 0.05f; // Sentido anti-horário
                }
                
                // Normalizar ângulo
                while (shared_game_state.aim_theta > 2.0f * M_PI) {
                    shared_game_state.aim_theta -= 2.0f * M_PI;
                }
                while (shared_game_state.aim_theta < 0.0f) {
                    shared_game_state.aim_theta += 2.0f * M_PI;
                }
                
                // Detectar disparo: quando potência cai rapidamente (soltou a pressão)
                // e havia potência suficiente antes
                float power_diff = last_power - shared_game_state.power;
                if (power_diff > 0.2f && max_power > 0.15f && !shared_game_state.shooting) {
                    // Disparar com a potência máxima alcançada
                    shared_game_state.ball_vx = max_power * 200.0f * 
                                                cosf(shared_game_state.aim_theta);
                    shared_game_state.ball_vy = max_power * 200.0f * 
                                                sinf(shared_game_state.aim_theta);
                    shared_game_state.shooting = 1;
                    shared_game_state.strokes++;
                    shared_game_state.power = 0.0f;
                    max_power = 0.0f;
                }
                
                // Atualizar máximo de potência
                if (shared_game_state.power > max_power) {
                    max_power = shared_game_state.power;
                }
                
                last_power = shared_game_state.power;
            } else {
                // Resetar quando não está atirando
                last_power = 0.0f;
                max_power = 0.0f;
            }
            
            // Reset do jogo se ambos botões pressionados
            if (btn_cw && btn_ccw && shared_game_state.game_over) {
                game_reset(&shared_game_state);
                last_power = 0.0f;
                max_power = 0.0f;
            }
            
            xSemaphoreGive(game_mutex);
        }
        
        // Temporização determinística
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// Inicialização das tarefas
void tasks_init(void) {
    // Criar objetos de sincronização
    load_cell_queue = xQueueCreate(10, sizeof(load_cell_data_t));
    render_semaphore = xSemaphoreCreateBinary();
    game_mutex = xSemaphoreCreateMutex();
    clock_mutex = xSemaphoreCreateMutex();
    display_mutex = xSemaphoreCreateMutex();
    
    // Criar tarefas
    xTaskCreate(task_hx711, "HX711", 256, NULL, 3, NULL);
    xTaskCreate(task_game_update, "GameUpdate", 512, NULL, 2, NULL);
    xTaskCreate(task_render, "Render", 512, NULL, 1, NULL);
    xTaskCreate(task_clock, "Clock", 128, NULL, 1, NULL);
    xTaskCreate(task_buttons, "Buttons", 256, NULL, 2, NULL);
    
    printf("Tarefas criadas\n");
}

