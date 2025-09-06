#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "serial.h"
#include "st7789.h"
#include "board.h"
#include "uart.h"

// Variáveis globais do jogo
static int ball_x = 120;  // Posição X da bolinha (centro inicial)
static int ball_y = 120;  // Posição Y da bolinha (centro inicial)
static int ball_dx = 2;   // Velocidade X da bolinha
static int ball_dy = 1;   // Velocidade Y da bolinha
static int ball_radius = 8; // Raio da bolinha

static uint32_t last_log_time = 0;  // Último tempo de log
static uint32_t last_color_change = 0; // Último tempo de mudança de cor
static uint16_t header_colors[] = {C_RED, C_GREEN, C_BLUE, C_YELL, C_CYAN, C_MAG};
static int current_color_index = 0;

// Função para desenhar a bolinha
static void draw_ball(void) {
    st7789_fill_circle(ball_x, ball_y, ball_radius, C_WHITE);
}

// Função para apagar a bolinha (desenhar com cor preta)
static void erase_ball(void) {
    st7789_fill_circle(ball_x, ball_y, ball_radius, C_BLACK);
}

// Função para desenhar o cabeçalho com uptime e quadrado colorido
static void draw_header(void) {
    uint32_t uptime_seconds = millis() / 1000;
    char uptime_str[20];
    sprintf(uptime_str, "Uptime: %lu s", uptime_seconds);
    
    // Desenhar fundo preto no cabeçalho
    st7789_fill_rect(0, 0, LCD_W, 30, C_BLACK);
    
    // Desenhar texto do uptime
    st7789_draw_text_5x7(5, 5, uptime_str, C_WHITE, 1, 0, 0);
    
    // Desenhar quadrado colorido no canto superior direito
    uint16_t current_color = header_colors[current_color_index];
    st7789_fill_rect(LCD_W - 25, 5, 20, 20, current_color);
}

// Função para atualizar a posição da bolinha
static void update_ball_position(void) {
    // Apagar bolinha na posição atual
    erase_ball();
    
    // Atualizar posição
    ball_x += ball_dx;
    ball_y += ball_dy;
    
    // Verificar colisões com as bordas
    if (ball_x - ball_radius <= 0 || ball_x + ball_radius >= LCD_W) {
        ball_dx = -ball_dx;
        // Ajustar posição para não sair da tela
        if (ball_x - ball_radius <= 0) ball_x = ball_radius;
        if (ball_x + ball_radius >= LCD_W) ball_x = LCD_W - ball_radius;
    }
    
    if (ball_y - ball_radius <= 30 || ball_y + ball_radius >= LCD_H) {
        ball_dy = -ball_dy;
        // Ajustar posição para não sair da tela (considerando cabeçalho)
        if (ball_y - ball_radius <= 30) ball_y = 30 + ball_radius;
        if (ball_y + ball_radius >= LCD_H) ball_y = LCD_H - ball_radius;
    }
    
    // Desenhar bolinha na nova posição
    draw_ball();
}

// Função para verificar mudança de cor do quadrado
static void check_color_change(void) {
    uint32_t current_time = millis();
    if (current_time - last_color_change >= 1000) { // 1 segundo
        current_color_index = (current_color_index + 1) % 6;
        last_color_change = current_time;
        draw_header(); // Redesenhar cabeçalho com nova cor
    }
}

// Função para imprimir log na serial
static void print_log(void) {
    uint32_t current_time = millis();
    if (current_time - last_log_time >= 200) { // 200 ms
        printf("Ball: x=%d, y=%d, Uptime: %lu ms\r\n", ball_x, ball_y, current_time);
        last_log_time = current_time;
    }
}

// Função para processar comandos da serial
static void process_serial_commands(void) {
    if (uart_rx_ready()) {
        char c = uart_getc();
        switch(c) {
            case '+':
                // Aumentar velocidade
                if (ball_dx < 0) ball_dx--;
                else ball_dx++;
                if (ball_dy < 0) ball_dy--;
                else ball_dy++;
                printf("Velocidade aumentada: dx=%d, dy=%d\r\n", ball_dx, ball_dy);
                break;
            case '-':
                // Diminuir velocidade (mínimo 1)
                if (abs(ball_dx) > 1) {
                    if (ball_dx < 0) ball_dx++;
                    else ball_dx--;
                }
                if (abs(ball_dy) > 1) {
                    if (ball_dy < 0) ball_dy++;
                    else ball_dy--;
                }
                printf("Velocidade diminuída: dx=%d, dy=%d\r\n", ball_dx, ball_dy);
                break;
            case 'r':
            case 'R':
                // Reset do jogo
                ball_x = 120;
                ball_y = 120;
                ball_dx = 2;
                ball_dy = 1;
                printf("Jogo resetado\r\n");
                break;
            default:
                break;
        }
    }
}

// Função principal do game demo
void demo_game(void) {
    printf("[demo 10] Game demo iniciado\r\n");
    printf("Controles: + (aumentar velocidade), - (diminuir velocidade), r (reset)\r\n");
    
    // Limpar tela
    st7789_fill_screen(C_BLACK);
    
    // Inicializar variáveis
    ball_x = 120;
    ball_y = 120;
    ball_dx = 2;
    ball_dy = 1;
    last_log_time = millis();
    last_color_change = millis();
    current_color_index = 0;
    
    // Desenhar cabeçalho inicial
    draw_header();
    
    // Desenhar bolinha inicial
    draw_ball();
    
    // Loop principal do jogo
    for(;;) {
        // Processar comandos da serial
        process_serial_commands();
        
        // Atualizar posição da bolinha
        update_ball_position();
        
        // Verificar mudança de cor do quadrado
        check_color_change();
        
        // Imprimir log na serial
        print_log();
        
        // Pequeno delay para controlar a velocidade do jogo
        delay_ms(50);
    }
}
