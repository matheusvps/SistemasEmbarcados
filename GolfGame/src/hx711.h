#pragma once
#include <stdint.h>
#include "stm32f4xx.h"

// Pinos HX711 (configuráveis)
// PB10 - SCK HX711, PB3 - DT HX711
#define HX711_DT_PORT  GPIOB
#define HX711_DT_PIN   3
#define HX711_SCK_PORT GPIOB
#define HX711_SCK_PIN  10

// Calibração (ajustar conforme sua célula)
// Offset inicial 0: o offset real é aprendido via hx711_tare()
#define HX711_OFFSET       0
#define HX711_SCALE_FACTOR 1000.0f  // Fator de escala (ajustar com calibração)

// Código de erro para leitura bruta (por exemplo, timeout)
#define HX711_RAW_ERROR    ((int32_t)0x80000000)

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


