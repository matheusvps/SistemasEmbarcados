#include "tasks.h"
#include "stm32f4xx.h"
#include "mpu6050.h"
#include "board.h"
#include "serial.h"
#include "st7789.h"
#include "hx711.h"
#include <stdio.h>
#include <math.h>

// Objetos de sincronização
QueueHandle_t accel_queue;
SemaphoreHandle_t render_semaphore;
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

// Tarefa de atualização do jogo
void task_game_update(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(16); // ~60 FPS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    accel_data_t accel_data;
    float dt = 0.016f; // 16ms
    
    printf("Tarefa Game Update iniciada\n");
    
    // Inicializar estado do jogo
    if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
        shared_game_state.ball_x = 50.0f;
        shared_game_state.ball_y = 120.0f;
        shared_game_state.ball_vx = 0.0f;
        shared_game_state.ball_vy = 0.0f;
        shared_game_state.aim_theta = 0.0f;
        shared_game_state.power = 0.0f;
        shared_game_state.shooting = 0;
        shared_game_state.button_pressed = 0;
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
            // Verificar se soltou a célula de carga (disparar)
            // A tarefa load_cell já atualizou a potência, só precisamos disparar quando soltar
            static int was_pressing = 0;
            
            if (shared_game_state.button_pressed) {
                // Está pressionando - aguardar soltar
                was_pressing = 1;
            } else if (was_pressing && !shared_game_state.shooting) {
                // Acabou de soltar - disparar com a potência acumulada
                if (shared_game_state.power > 0.01f) {  // Mínimo de força
                    shared_game_state.ball_vx = shared_game_state.power * 200.0f * cosf(shared_game_state.aim_theta);
                    shared_game_state.ball_vy = shared_game_state.power * 200.0f * sinf(shared_game_state.aim_theta);
                    shared_game_state.shooting = 1;
                    shared_game_state.power = 0.0f;  // Resetar potência
                }
                was_pressing = 0;
            }
            
            // Atualizar posição da bola
            if (shared_game_state.shooting) {
                shared_game_state.ball_x += shared_game_state.ball_vx * dt;
                shared_game_state.ball_y += shared_game_state.ball_vy * dt;
                
                // Fricção
                shared_game_state.ball_vx *= 0.95f;
                shared_game_state.ball_vy *= 0.95f;
                
                // Parar se velocidade muito baixa
                if (fabsf(shared_game_state.ball_vx) < 1.0f && 
                    fabsf(shared_game_state.ball_vy) < 1.0f) {
                    shared_game_state.ball_vx = 0.0f;
                    shared_game_state.ball_vy = 0.0f;
                    shared_game_state.shooting = 0;
                }
                
                // Colisão com bordas
                if (shared_game_state.ball_x < 10.0f || shared_game_state.ball_x > 230.0f) {
                    shared_game_state.ball_vx = -shared_game_state.ball_vx * 0.5f;
                    if (shared_game_state.ball_x < 10.0f) shared_game_state.ball_x = 10.0f;
                    if (shared_game_state.ball_x > 230.0f) shared_game_state.ball_x = 230.0f;
                }
                if (shared_game_state.ball_y < 10.0f || shared_game_state.ball_y > 230.0f) {
                    shared_game_state.ball_vy = -shared_game_state.ball_vy * 0.5f;
                    if (shared_game_state.ball_y < 10.0f) shared_game_state.ball_y = 10.0f;
                    if (shared_game_state.ball_y > 230.0f) shared_game_state.ball_y = 230.0f;
                }
            }
            
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
    printf("Tarefa Render iniciada\n");
    
    for (;;) {
        // Aguardar semáforo de renderização
        if (xSemaphoreTake(render_semaphore, portMAX_DELAY) == pdTRUE) {
            // Obter estado do jogo
            game_state_t state;
            if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
                state = shared_game_state;
                xSemaphoreGive(game_mutex);
            } else {
                continue;
            }
            
            // Obter relógio
            clock_time_t clock;
            if (xSemaphoreTake(clock_mutex, 0) == pdTRUE) {
                clock = shared_clock;
                xSemaphoreGive(clock_mutex);
            }
            
            // Limpar tela (fundo verde)
            st7789_fill_screen(0x4354); // Verde RGB565
            
            // Desenhar buraco
            st7789_fill_circle(200, 120, 20, C_BLACK);
            
            // Desenhar bola
            st7789_fill_circle((int)state.ball_x, (int)state.ball_y, 8, C_WHITE);
            st7789_draw_circle((int)state.ball_x, (int)state.ball_y, 8, C_BLACK);
            
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
            
            // Desenhar relógio (hh:mm:ss)
            char time_str[9];
            snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", 
                    clock.hours, clock.minutes, clock.seconds);
            st7789_draw_text_5x7(150, 5, time_str, C_WHITE, 1, 1, C_BLACK);
        }
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
    
    // Aguardar um pouco para estabilizar
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Fazer tara (zerar)
    hx711_tare();
    printf("Tara concluída. Célula de carga pronta.\n");
    
    float max_force = 0.0f;  // Força máxima detectada nesta "pressionada"
    int is_pressing = 0;      // Flag se está pressionando
    
    for (;;) {
        // Ler força da célula de carga
        float force = hx711_read_weight();  // Força em kg
        
        // Threshold mínimo para detectar pressão (evitar ruído)
        const float MIN_FORCE = 0.05f;  // 50g mínimo
        
        if (xSemaphoreTake(game_mutex, portMAX_DELAY) == pdTRUE) {
            if (force > MIN_FORCE) {
                // Força detectada - está pressionando
                if (!is_pressing) {
                    // Começou a pressionar agora
                    is_pressing = 1;
                    max_force = force;
                    shared_game_state.button_pressed = 1;  // Sinalizar que começou
                } else {
                    // Continuando a pressionar - atualizar força máxima
                    if (force > max_force) {
                        max_force = force;
                    }
                    // Atualizar potência baseada na força atual
                    // Normalizar para 0-1 (assumindo célula de 1kg)
                    shared_game_state.power = max_force / 1.0f;
                    if (shared_game_state.power > 1.0f) {
                        shared_game_state.power = 1.0f;
                    }
                    shared_game_state.button_pressed = 1;  // Mantém pressionado
                }
            } else {
                // Sem força - parou de pressionar
                if (is_pressing) {
                    // Acabou de soltar - a potência já está atualizada
                // O disparo será feito na tarefa game_update quando detectar button_pressed = 0
                is_pressing = 0;
                max_force = 0.0f;
                shared_game_state.button_pressed = 0;
                }
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
    accel_queue = xQueueCreate(5, sizeof(accel_data_t));
    render_semaphore = xSemaphoreCreateBinary();
    game_mutex = xSemaphoreCreateMutex();
    clock_mutex = xSemaphoreCreateMutex();
    
    // Criar tarefas
    xTaskCreate(task_mpu6050, "MPU6050", 256, NULL, 3, NULL);
    xTaskCreate(task_game_update, "GameUpdate", 512, NULL, 2, NULL);
    xTaskCreate(task_render, "Render", 512, NULL, 1, NULL);
    xTaskCreate(task_clock, "Clock", 128, NULL, 2, NULL);
    xTaskCreate(task_button, "LoadCell", 256, NULL, 2, NULL);  // Aumentado stack para HX711
    
    printf("Tarefas criadas\n");
}

