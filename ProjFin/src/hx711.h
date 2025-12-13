#pragma once
#include <stdint.h>
#include "stm32f4xx.h"

// Pinos HX711 (configuráveis)
// NOTA: PA7 é usado pelo SPI1 (LCD), então usamos PA8 para SCK
#define HX711_DT_PORT  GPIOA
#define HX711_DT_PIN   6
#define HX711_SCK_PORT GPIOA
#define HX711_SCK_PIN  8

// Calibração (ajustar conforme sua célula)
#define HX711_OFFSET       8388608  // Offset padrão (meio da escala 24-bit)
#define HX711_SCALE_FACTOR 1000.0f  // Fator de escala (ajustar com calibração)

// Funções de inicialização
void hx711_init(void);

// Leitura de dados
int32_t hx711_read_raw(void);
float hx711_read_weight(void);  // Retorna peso em kg

// Calibração
void hx711_set_offset(int32_t offset);
void hx711_set_scale(float scale);
void hx711_tare(void);  // Zerar a balança (tara)

// Utilitários
int hx711_is_ready(void);

