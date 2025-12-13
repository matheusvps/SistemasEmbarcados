#include "stm32f4xx.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

/* Configura SysTick - será usado pelo FreeRTOS */
void delay_init(void){
    SystemCoreClockUpdate();
    // O FreeRTOS configurará o SysTick automaticamente
}

/* Delay usando FreeRTOS */
void delay_ms(uint32_t ms){
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        // Antes do scheduler iniciar, usar delay simples
        volatile uint32_t count = ms * 1000;
        while (count--);
    }
}

uint32_t millis(void){
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        return xTaskGetTickCount();
    }
    return 0;
}
