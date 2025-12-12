# Projeto Final - Mini-Golf no STM32F411 com FreeRTOS

## Descrição
Implementação completa do jogo Mini-Golf para STM32F411 BlackPill com display LCD ST7789 240x240, usando FreeRTOS com múltiplas tarefas.

## Requisitos Implementados

✅ **Mínimo 3 threads/tarefas**: 5 tarefas implementadas
- Tarefa MPU6050 (leitura do acelerômetro)
- Tarefa Game Update (atualização da física)
- Tarefa Render (renderização no LCD)
- Tarefa Clock (atualização do relógio)
- Tarefa Button (leitura do botão)

✅ **Máquina de estados**: Implementada na tarefa MPU6050
- Estados: INIT, WAIT, READ, ERROR
- Tratamento de erros e reinicialização

✅ **Objetos de sincronização**:
- **Fila**: `accel_queue` - Comunicação entre MPU6050 e Game Update
- **Mutex**: `game_mutex` - Proteção do estado do jogo
- **Mutex**: `clock_semaphore` - Proteção do relógio
- **Semáforo**: `render_semaphore` - Sincronização de renderização

✅ **Temporização determinística**:
- Uso de `vTaskDelayUntil()` em todas as tarefas
- Uso de `xTaskGetTickCount()` para timing
- Relógio atualizado a cada segundo

✅ **Relógio hh:mm:ss**: Exibido no canto superior direito do display

## Controles

- **Acelerômetro MPU6050**: Define a direção da bolinha (inclinação da placa)
- **Botão PA10**: Dispara a bolinha (mantenha pressionado para carregar potência)

## Estrutura do Projeto

```
ProjetoFinal/
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
cd ProjetoFinal

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
cd ProjetoFinal
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

### Botão
- Um terminal → PA10
- Outro terminal → GND
- (Pull-up interno habilitado)

## Funcionamento

1. **Tarefa MPU6050**: Lê dados do acelerômetro a 50 Hz e envia para a fila
2. **Tarefa Game Update**: Processa física do jogo a ~60 FPS, usa dados do acelerômetro para direção
3. **Tarefa Render**: Renderiza o jogo no LCD quando recebe semáforo
4. **Tarefa Clock**: Atualiza relógio a cada segundo
5. **Tarefa Button**: Lê botão a 20 Hz e atualiza estado

## Características do Jogo

- Bola branca controlada pelo acelerômetro
- Buraco preto na posição (200, 120)
- Linha de mira mostra direção do tiro
- Barra de potência vermelha (carrega enquanto botão pressionado)
- Relógio hh:mm:ss no canto superior direito
- Física com fricção e colisões com bordas

## Debug

A comunicação serial (115200 baud) mostra mensagens de inicialização e status das tarefas.
