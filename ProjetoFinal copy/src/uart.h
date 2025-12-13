#pragma once
#include "stm32f4xx.h"

static inline int uart_rx_ready(void) { 
    return (USART1->SR & USART_SR_RXNE) != 0; 
}

static inline char uart_getc(void) { 
    return (char)USART1->DR; 
}

