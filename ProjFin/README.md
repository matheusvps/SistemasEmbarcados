# Projeto Final - Mini Golf com Sensor de Carga

## Descrição
Implementação completa do jogo Mini-Golf para STM32F411 BlackPill com display LCD ST7789 240x240, usando FreeRTOS com múltiplas tarefas e sensor de carga HX711 para controlar a força da tacada.

## Requisitos Implementados

✅ **Mínimo 3 threads/tarefas**: 5 tarefas implementadas
- Tarefa HX711 (leitura do sensor de carga) - 50 Hz
- Tarefa Game Update (atualização da física) - ~60 FPS
- Tarefa Render (renderização no LCD) - ~30 FPS
- Tarefa Clock (atualização do relógio) - 1 Hz
- Tarefa Buttons (leitura dos botões) - 20 Hz

✅ **Máquina de estados**: Implementada na tarefa HX711
- Estados: INIT, WAIT, READ, ERROR
- Tratamento de erros e reinicialização automática

✅ **Objetos de sincronização**:
- **Fila**: `load_cell_queue` - Comunicação entre HX711 e Game Update
- **Mutex**: `game_mutex` - Proteção do estado do jogo
- **Mutex**: `clock_mutex` - Proteção do relógio
- **Mutex**: `display_mutex` - Proteção do display
- **Semáforo**: `render_semaphore` - Sincronização de renderização

✅ **Temporização determinística**:
- Uso de `vTaskDelayUntil()` em todas as tarefas
- Uso de `xTaskGetTickCount()` para timing
- Relógio atualizado a cada segundo

✅ **Relógio hh:mm:ss**: Exibido no canto superior direito do display

## Controles

- **Botão PA9 (CW)**: Gira a mira no sentido horário / Dispara a bola quando pressionado com potência
- **Botão PA10 (CCW)**: Gira a mira no sentido anti-horário
- **Sensor de Carga HX711**: Controla a força da tacada (pressão aplicada no sensor)

## Estrutura do Projeto

```
ProjFin/
├── src/
│   ├── main.c              # Arquivo principal, inicia FreeRTOS
│   ├── tasks.c/h           # Tarefas FreeRTOS
│   ├── game.c/h             # Lógica do jogo Mini Golf
│   ├── hx711.c/h            # Driver do sensor HX711
│   ├── board.h              # Configurações de hardware
│   ├── delay.c/h            # Funções de delay (compatível FreeRTOS)
│   ├── serial.c/h           # Comunicação serial
│   ├── st7789.c/h           # Driver do display LCD
│   ├── font5x7.h            # Fonte para texto
│   ├── uart.h               # Helpers UART
│   └── FreeRTOSConfig.h     # Configuração do FreeRTOS
├── platformio.ini           # Configuração PlatformIO
└── README.md                 # Este arquivo
```

## Compilação

### Usando PlatformIO

```bash
cd ProjFin

# Compilar
pio run -e blackpill_f411ce

# Fazer upload
pio run -e blackpill_f411ce -t upload

# Monitorar serial
pio device monitor
```

## Conexões

### Display LCD (ST7789)
- DC → PB0
- RST → PB1
- BLK → PB6
- CS → PB10
- SCK → PA5
- MOSI → PA7

### Sensor de Carga HX711
- VCC → 3.3V ou 5V
- GND → GND
- DT → PA6
- SCK → PA8

### Botões
- Botão CW (horário) → PA9 (outro terminal → GND, pull-up interno)
- Botão CCW (anti-horário) → PA10 (outro terminal → GND, pull-up interno)

## Funcionamento

1. **Tarefa HX711**: Lê dados do sensor de carga a 50 Hz e envia para a fila
2. **Tarefa Game Update**: Processa física do jogo a ~60 FPS, usa dados do sensor para potência
3. **Tarefa Render**: Renderiza o jogo no LCD quando recebe semáforo
4. **Tarefa Clock**: Atualiza relógio a cada segundo
5. **Tarefa Buttons**: Lê botões a 20 Hz e atualiza direção/disparo

## Características do Jogo

- Bola branca controlada pela física
- Buraco preto na posição (200, 120)
- Linha de mira mostra direção do tiro (ciano)
- Barra de potência vermelha (atualiza em tempo real conforme pressão no sensor)
- Relógio hh:mm:ss no canto superior direito
- Contador de tacadas (strokes)
- Física com fricção e colisões com bordas
- Mensagem "HOLE IN ONE!" quando a bola cai no buraco

## Como Jogar

1. Use os botões PA9 (horário) e PA10 (anti-horário) para ajustar a direção da mira
2. Aplique pressão no sensor de carga HX711 para carregar a potência (barra vermelha)
3. Pressione o botão PA9 enquanto aplica pressão no sensor para disparar
4. A bola se move com física realista até parar ou cair no buraco
5. O objetivo é acertar o buraco com o menor número de tacadas possível

## Debug

A comunicação serial (115200 baud) mostra mensagens de inicialização e status das tarefas.

## Calibração do Sensor HX711

O sensor pode precisar de calibração. Ajuste os valores em `hx711.h`:
- `HX711_OFFSET`: Offset para zerar a balança
- `HX711_SCALE_FACTOR`: Fator de escala para converter valores brutos em força

Use a função `hx711_tare()` para fazer tara (zerar) quando não houver pressão aplicada.

