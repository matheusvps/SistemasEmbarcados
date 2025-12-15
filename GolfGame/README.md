# GolfGame - Mini-Golf no STM32F411 com FreeRTOS

## Descrição
Implementação completa do jogo Mini-Golf para STM32F411 BlackPill com display LCD ST7789 240x240, usando FreeRTOS com múltiplas tarefas. Direção pela inclinação (MPU6050), potência pela célula de carga HX711 e disparo via botão dedicado.

## Requisitos Implementados

✅ **Mínimo 3 threads/tarefas**: 4 tarefas implementadas
- Tarefa MPU6050 (leitura do acelerômetro)
- Tarefa Game Update (atualização da física + renderização no LCD)
- Tarefa Clock (atualização do relógio)
- Tarefa Button (leitura do botão)

✅ **Máquina de estados**: Implementada na tarefa MPU6050
- Estados: INIT, WAIT, READ, ERROR
- Tratamento de erros e reinicialização

✅ **Objetos de sincronização**:
- **Fila**: `accel_queue` - Comunicação entre MPU6050 e Game Update
- **Mutex**: `game_mutex` - Proteção do estado do jogo
- **Mutex**: `clock_mutex` - Proteção do relógio

✅ **Temporização determinística**:
- Uso de `vTaskDelayUntil()` em todas as tarefas
- Uso de `xTaskGetTickCount()` para timing
- Relógio atualizado a cada segundo

✅ **Relógio hh:mm:ss**: Exibido no canto superior direito do display

## Controles

- **Acelerômetro MPU6050**: Define a direção da bolinha (inclinação da placa)
- **Célula de Carga HX711**: Mede a força aplicada para controlar a potência do tiro
  - Aplique força na célula; potência em tempo real
- **Botão (PA10)**: Dispara a bola usando a potência corrente

## Estrutura do Projeto

```
GolfGame/
├── src/
│   ├── main.c              # Arquivo principal, inicia FreeRTOS
│   ├── tasks.c/h           # Tarefas FreeRTOS
│   ├── board.h             # Configurações de hardware
│   ├── delay.c/h           # Funções de delay (compatível FreeRTOS)
│   ├── serial.c/h          # Comunicação serial
│   ├── st7789.c/h          # Driver do display LCD
│   ├── mpu6050.c/h         # Driver do sensor MPU6050
│   ├── font5x7.h           # Fonte para texto
│   ├── uart.h              # Helpers UART
│   └── FreeRTOSConfig.h    # Configuração do FreeRTOS
├── platformio.ini          # Configuração PlatformIO
├── Makefile                # Makefile para compilação e upload
└── README.md               # Este arquivo
```

## Compilação

### Usando Makefile (Recomendado)

```bash
cd GolfGame

# Compilar, limpar e fazer upload (padrão)
make

# Apenas compilar
make build

# Apenas fazer upload
make upload

# Compilar e fazer upload
make build-upload

# Monitorar serial
make monitor

# Compilar, fazer upload e monitorar
make deploy

# Ver ajuda completa
make help
```

### Usando PlatformIO diretamente

```bash
cd GolfGame
pio run -e blackpill_f411ce
pio run -e blackpill_f411ce -t upload
```

## Conexões

### Display LCD (ST7789)
- DC → PB0
- RST → PB1
- BLK → PB6
- CS → PB10
- SCK → PA5
- MOSI → PA7

### MPU6050
- VCC → 3.3V
- GND → GND
- SCL → PB8 (I2C1_SCL)
- SDA → PB9 (I2C1_SDA)

### Célula de Carga HX711
- **HX711 VCC** → 3V3
- **HX711 GND** → GND
- **HX711 DT** → PA1
- **HX711 SCK** → PA2
- **Célula de Carga** → HX711 (E+, E-, S+, S-)

### Pushbutton de Lançamento
- **PB7** → Pushbutton (entrada com pull‑up interno, usado para disparar a tacada)

**Nota**: PA7 é usado pelo SPI1 (LCD), então usamos PA8 para SCK do HX711.

**Componentes necessários**:
- Módulo HX711 (R$ 5-15)
- Célula de carga 1kg (R$ 15-30)

Ver `CELULA_CARGA_GUIA.md` para detalhes completos.

## Funcionamento

1. **Tarefa MPU6050**: Lê dados do acelerômetro a 50 Hz e envia para a fila
2. **Tarefa Game Update**: Processa física do jogo a ~60 FPS e desenha tudo no LCD
3. **Tarefa Clock**: Atualiza relógio a cada segundo
4. **Tarefa LoadCell/Button**: Lê célula de carga a 20 Hz, atualiza potência e verifica botão de disparo

## Tratamento dos Inputs

- **Acelerômetro (MPU6050)**  
  - A tarefa `task_mpu6050` lê os registradores brutos do MPU6050, converte para unidades físicas (g e dps) e envia uma estrutura `accel_data_t` para a fila `accel_queue`.  
  - A tarefa `task_game_update` consome a fila (não bloqueante) e, quando a bola não está em movimento (`shooting == 0`), converte `ax`/`ay` em um ângulo usando `atan2f(ay, ax)` e atualiza `shared_game_state.aim_theta`.  
  - Esse ângulo é usado para desenhar a linha de mira e para calcular o vetor de velocidade da tacada quando o disparo acontece.

- **Célula de carga (HX711)**  
  - A tarefa `task_button` inicializa o HX711 (`hx711_init`, `hx711_tare`) e periodicamente chama `hx711_read_weight()` para obter o peso/força aplicada em kg.  
  - Um limiar mínimo é usado para filtrar ruído; acima desse valor a força é normalizada para o intervalo \[0, 1\] e gravada em `shared_game_state.power`, protegida por `game_mutex`.  
  - A barra de potência no LCD é desenhada em `task_game_update` lendo esse campo do estado compartilhado, atualizando em tempo real conforme a pressão na célula.

- **Push button de lançamento**  
  - O botão físico em `PA10` é configurado com pull‑up interno em `button_init()` e lido pela função `button_is_pressed()`, que é chamada dentro de `task_button`.  
  - `task_button` faz **debounce por software** em alta taxa (100 Hz) e gera um evento de clique (`button_event`), protegido por `game_mutex`, garantindo que cliques rápidos não sejam perdidos. O estado estável atual do botão fica em `shared_game_state.button_pressed`.  
  - Em `task_game_update`, o evento de clique (`button_event`) é consumido sempre que a bola não está se movendo e o buraco ainda não foi concluído; a tarefa usa a potência atual (`power`) junto com `aim_theta` para calcular `ball_vx` e `ball_vy`, incrementa o contador de tacadas (`strokes`), marca `shooting = 1` e limpa o evento.

## Características do Jogo

- Bola branca controlada pelo acelerômetro
- Buraco em (200, 120) com contador de tacadas e status de conclusão
- Obstáculos retangulares com colisão amortecida
- Linha de mira mostra direção do tiro
- Barra de potência vermelha (tempo real)
- Relógio hh:mm:ss no canto superior direito
- Física com fricção e colisões com bordas/obstáculos

## Debug

A comunicação serial (115200 baud) mostra mensagens de inicialização e status das tarefas.
