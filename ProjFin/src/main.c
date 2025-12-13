#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "delay.h"
#include "st7789.h"
#include "board.h"
#include "tasks.h"
#include <stdio.h>

// Configurações
#define SERIAL_BAUDRATE 115200

int main(void) {
    // Inicialização básica do sistema
    SystemCoreClockUpdate();
    delay_init();
    serial_stdio_init(SERIAL_BAUDRATE);
    
    printf("\n=== Mini-Golf STM32F411 ===\n");
    printf("Inicializando sistema...\n");
    
    // Inicializar display LCD
    printf("Inicializando LCD...\n");
    st7789_init();
    st7789_set_speed_div(0);
    st7789_fill_screen(C_BLACK);
    delay_ms(100);
    
    // Inicializar tarefas e objetos de sincronização
    printf("Criando tarefas FreeRTOS...\n");
    tasks_init();
    
    printf("Iniciando scheduler FreeRTOS...\n");
    
    // Iniciar o scheduler do FreeRTOS
    vTaskStartScheduler();
    
    // Nunca deve chegar aqui
    for (;;) {
        // Se o scheduler falhar, piscar LED (PC13)
        GPIOC->ODR ^= (1u << 13);
        delay_ms(500);
    }
    
    return 0;
}

