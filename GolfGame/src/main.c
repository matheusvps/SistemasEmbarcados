#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "delay.h"
#include "st7789.h"
#include "board.h"
#include "mpu6050.h"
#include "hx711.h"
#include "tasks.h"
#include <stdio.h>

// Configurações
#define SERIAL_BAUDRATE 115200
#define I2C_CLOCK_SPEED 100000

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
    st7789_fill_screen(C_BLACK);
    delay_ms(100);
    
    // Inicializar I2C para MPU6050
    printf("Inicializando I2C...\n");
    i2c1_init_100k(I2C_CLOCK_SPEED);
    
    // Inicializar tarefas e objetos de sincronização
    printf("Criando tarefas FreeRTOS...\n");
    tasks_init();
    
    printf("Iniciando scheduler FreeRTOS...\n");
    
    // Iniciar o scheduler do FreeRTOS
    vTaskStartScheduler();
    
    // Nunca deve chegar aqui
    for (;;) {
        // Se o scheduler falhar, piscar LED
        GPIOC->ODR ^= (1u << 13);
        delay_ms(500);
    }
    
    return 0;
}

// Handler do SysTick (usado pelo FreeRTOS)
void xPortSysTickHandler(void);

// Handler do SysTick precisa chamar o do FreeRTOS
void SysTick_Handler(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
