#include "hx711.h"
#include "stm32f4xx.h"
#include <stdint.h>

// Variáveis de calibração
static int32_t hx711_offset = HX711_OFFSET;
static float hx711_scale = HX711_SCALE_FACTOR;

// Helpers para GPIO
static inline void pin_set(GPIO_TypeDef* port, uint32_t pin) {
    port->BSRR = (1u << pin);
}

static inline void pin_clr(GPIO_TypeDef* port, uint32_t pin) {
    port->BSRR = (1u << (pin + 16));
}

static inline int pin_read(GPIO_TypeDef* port, uint32_t pin) {
    return (port->IDR & (1u << pin)) ? 1 : 0;
}

// Inicialização do HX711
void hx711_init(void) {
    // Habilitar clock do GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    
    // Configurar DT como entrada (pull-up)
    GPIOB->MODER &= ~(3u << (HX711_DT_PIN * 2));
    GPIOB->PUPDR &= ~(3u << (HX711_DT_PIN * 2));
    GPIOB->PUPDR |= (1u << (HX711_DT_PIN * 2));
    
    // Configurar SCK como saída (inicialmente LOW)
    GPIOB->MODER &= ~(3u << (HX711_SCK_PIN * 2));
    GPIOB->MODER |= (1u << (HX711_SCK_PIN * 2));
    pin_clr(HX711_SCK_PORT, HX711_SCK_PIN);
    
    // Aguardar HX711 ficar pronto
    for (volatile int i = 0; i < 1000; i++);
    
    // Fazer tara inicial
    hx711_tare();
}

// Verifica se HX711 está pronto (DT em LOW)
int hx711_is_ready(void) {
    return !pin_read(HX711_DT_PORT, HX711_DT_PIN);
}

// Lê valor bruto de 24 bits do HX711
int32_t hx711_read_raw(void) {
    int32_t value = 0;
    
    // Aguardar HX711 ficar pronto
    while (!hx711_is_ready()) {
        // Timeout - retornar 0 se demorar muito
        volatile int timeout = 10000;
        if (--timeout == 0) return 0;
    }
    
    // Ler 24 bits (MSB first)
    for (int i = 0; i < 24; i++) {
        // Clock HIGH
        pin_set(HX711_SCK_PORT, HX711_SCK_PIN);
        
        // Pequeno delay
        for (volatile int j = 0; j < 2; j++);
        
        // Ler bit
        value = value << 1;
        if (pin_read(HX711_DT_PORT, HX711_DT_PIN)) {
            value++;
        }
        
        // Clock LOW
        pin_clr(HX711_SCK_PORT, HX711_SCK_PIN);
        
        // Pequeno delay
        for (volatile int j = 0; j < 2; j++);
    }
    
    // Configurar canal e ganho (canal A, ganho 128)
    pin_set(HX711_SCK_PORT, HX711_SCK_PIN);
    for (volatile int j = 0; j < 2; j++);
    pin_clr(HX711_SCK_PORT, HX711_SCK_PIN);
    
    // Converter de complemento de 2 para signed
    if (value & 0x800000) {
        value |= 0xFF000000;  // Estender sinal negativo
    }
    
    return value;
}

// Lê peso calibrado em kg
float hx711_read_weight(void) {
    int32_t raw = hx711_read_raw();
    if (raw == 0) return 0.0f;  // Erro na leitura
    
    // Aplicar offset e escala
    float weight = ((float)(raw - hx711_offset)) / hx711_scale;
    
    // Retornar apenas valores positivos (força aplicada)
    if (weight < 0.0f) weight = 0.0f;
    
    return weight;
}

// Configurar offset
void hx711_set_offset(int32_t offset) {
    hx711_offset = offset;
}

// Configurar fator de escala
void hx711_set_scale(float scale) {
    hx711_scale = scale;
}

// Fazer tara (zerar a balança)
void hx711_tare(void) {
    // Ler várias amostras e calcular média
    int32_t sum = 0;
    const int samples = 10;
    
    for (int i = 0; i < samples; i++) {
        int32_t raw = hx711_read_raw();
        if (raw != 0) {
            sum += raw;
        }
        // Pequeno delay entre leituras
        for (volatile int j = 0; j < 1000; j++);
    }
    
    hx711_offset = sum / samples;
}

