#include "stm32f4xx.h"
#include "serial.h"

#include <stdio.h>       // setvbuf
#include <sys/unistd.h>  // _write

/* ================= UART1 low-level helpers ================= */
static inline void uart1_putc_ll(uint8_t c) {
    while ((USART1->SR & USART_SR_TXE) == 0u) { /* wait */ }
    USART1->DR = c;
}
static inline int uart1_txc_done_ll(void) {
    return (USART1->SR & USART_SR_TC) != 0;
}

/* Calcula BRR para OVER8=0 (oversampling 16). */
static uint32_t usart1_brr_from_clk(uint32_t pclk, uint32_t baud){
    uint32_t div16 = (pclk + (baud/2u)) / baud;
    uint32_t mant  = div16 / 16u;
    uint32_t frac  = div16 % 16u;
    return (mant << 4) | (frac & 0xFu);
}

/* ================= API simples ================= */
void serial_init(uint32_t baud){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->MODER   &= ~((3u<<(9*2)) | (3u<<(10*2)));
    GPIOA->MODER   |=  ((2u<<(9*2)) | (2u<<(10*2)));
    GPIOA->PUPDR   &= ~((3u<<(9*2)) | (3u<<(10*2)));
    GPIOA->OSPEEDR |=  (2u<<(9*2));
    GPIOA->AFR[1]  &= ~((0xFu<<((9-8)*4)) | (0xFu<<((10-8)*4)));
    GPIOA->AFR[1]  |=  ((7u  <<((9-8)*4)) | (7u  <<((10-8)*4)));

    USART1->CR1 = 0u;
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;

    uint32_t pclk2 = SystemCoreClock;
    USART1->BRR = usart1_brr_from_clk(pclk2, baud);

    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;
}

void serial_write(const char *s){
    while (*s) {
        char c = *s++;
        if (c == '\n') uart1_putc_ll('\r');
        uart1_putc_ll((uint8_t)c);
    }
}

void serial_putc(uint8_t c){ uart1_putc_ll(c); }
int  serial_tx_done(void)  { return uart1_txc_done_ll(); }
int  serial_readable(void) { return (USART1->SR & USART_SR_RXNE) != 0; }

int  serial_getc_blocking(void){
    while (!serial_readable()) { }
    return (int)(USART1->DR & 0xFF);
}

void serial_stdio_init(uint32_t baud){
    serial_init(baud);
    setvbuf(stdout, NULL, _IONBF, 0);
}

int _write(int fd, const void *buf, size_t count) {
    (void)fd;
    const uint8_t *p = (const uint8_t*)buf;
    for (size_t i = 0; i < count; i++) {
        uint8_t c = p[i];
        if (c == '\n') uart1_putc_ll('\r');
        uart1_putc_ll(c);
    }
    while (!uart1_txc_done_ll()) {}
    return (int)count;
}

__attribute__((weak)) int _read(int fd, void *buf, size_t count) {
    (void)fd; (void)buf; (void)count; return 0;
}
__attribute__((weak)) caddr_t _sbrk(int incr) {
    extern uint8_t _end;
    static uint8_t *heap_end;
    uint8_t *prev;
    if (heap_end == 0) heap_end = &_end;
    prev = heap_end;
    heap_end += incr;
    return (caddr_t)prev;
}

