# Lab2 - Sistema de Monitoramento de Movimento com Comunicação Wireless

## Visão Geral

Este projeto implementa um sistema de monitoramento de movimento em tempo real utilizando um microcontrolador STM32F411 (BlackPill) equipado com sensor MPU6050 e módulo de comunicação wireless HC-12. O sistema coleta dados de aceleração e giroscópio e os transmite via comunicação serial para um receptor ESP32 que processa e analisa os dados para detectar eventos de movimento específicos.

## Arquitetura do Sistema

O projeto é composto por duas partes principais:

### 1. Transmissor (STM32F411)
- **Hardware**: STM32F411 BlackPill
- **Sensores**: MPU6050 (acelerômetro e giroscópio)
- **Comunicação**: Módulo HC-12 (433MHz)
- **Função**: Coleta dados do sensor e transmite via wireless

### 2. Receptor (ESP32)
- **Hardware**: ESP32 DevKit
- **Comunicação**: Módulo HC-12 (433MHz)
- **Função**: Recebe dados, processa e detecta eventos de movimento

## Funcionalidades Implementadas

### Transmissor (STM32F411)
- ✅ Inicialização e configuração do sensor MPU6050 via I2C
- ✅ Leitura contínua de dados de aceleração e giroscópio
- ✅ Configuração e comunicação via módulo HC-12
- ✅ Transmissão de dados em formato CSV via wireless
- ✅ LED de status indicando funcionamento do sistema
- ✅ Log serial para debug e monitoramento

### Receptor (ESP32)
- ✅ Recepção de dados via módulo HC-12
- ✅ Conversão de dados brutos para valores físicos
- ✅ Detecção de eventos específicos:
  - **Batida**: Aceleração Y < -1.2g
  - **Freada Brusca**: Aceleração Y < -0.8g
  - **Curva Acentuada**: Velocidade angular Z > 200°/s
- ✅ Saída serial para monitoramento dos eventos detectados

## Estrutura do Projeto

```
Lab2/
├── src/                    # Código fonte STM32F411
│   ├── main.c             # Arquivo principal
│   ├── mpu6050.c/h        # Driver do sensor MPU6050
│   ├── hc12.c/h           # Driver do módulo HC-12
│   ├── board.h            # Configurações de hardware
│   ├── delay.c/h          # Funções de delay
│   ├── serial.c/h         # Comunicação serial
│   ├── st7789.c/h         # Driver do display (não usado neste lab)
│   └── uart.h             # Configurações UART
├── esp/                   # Código fonte ESP32
│   └── src/
│       └── main.ino       # Código Arduino para ESP32
├── platformio.ini         # Configuração PlatformIO
└── README.md              # Este arquivo
```

## Hardware Necessário

### Transmissor
- STM32F411 BlackPill
- Sensor MPU6050
- Módulo HC-12
- Protoboard e jumpers
- Fonte de alimentação 5V

### Receptor
- ESP32 DevKit
- Módulo HC-12
- Protoboard e jumpers
- Fonte de alimentação 5V

## Conexões

### STM32F411 + MPU6050
- **VCC** → 3.3V
- **GND** → GND
- **SCL** → PB8 (I2C1_SCL)
- **SDA** → PB9 (I2C1_SDA)

### STM32F411 + HC-12
- **VCC** → 5V
- **GND** → GND
- **TX** → PA3 (USART2_RX)
- **RX** → PA2 (USART2_TX)
- **SET** → PA1 (GPIO)

### ESP32 + HC-12
- **VCC** → 5V
- **GND** → GND
- **TX** → GPIO17 (TX2)
- **RX** → GPIO16 (RX2)
- **SET** → GPIO5

## Compilação e Upload

### STM32F411 (Transmissor)
```bash
# Usando PlatformIO
pio run -e blackpill_f411ce
pio run -e blackpill_f411ce -t upload
```

### ESP32 (Receptor)
```bash
# Usando PlatformIO
cd esp/
pio run
pio run -t upload
```

## Formato dos Dados

### Transmissão (STM32F411 → ESP32)
```
ax,ay,az,gx,gy,gz
```
- **ax, ay, az**: Valores brutos do acelerômetro (16-bit signed)
- **gx, gy, gz**: Valores brutos do giroscópio (16-bit signed)

### Saída do Receptor (ESP32 → Serial)
```
1.23,-0.45,0.12    # Valores de aceleração em g
BATIDA             # Evento detectado
```

## Configurações

### MPU6050
- **Endereço I2C**: 0x68
- **Faixa do acelerômetro**: ±2g
- **Faixa do giroscópio**: ±250°/s
- **Taxa de amostragem**: ~50Hz

### HC-12
- **Baudrate**: 9600
- **Canal**: Padrão (433.4MHz)
- **Potência**: Padrão (20dBm)
- **Modo**: Transparente

## Monitoramento

### Serial do Transmissor (115200 baud)
```
ax,ay,az,gx,gy,gz
1234,567,890,123,456,789
...
```

### Serial do Receptor (115200 baud)
```
1.23,-0.45,0.12
BATIDA
FREADA BRUSCA
CURVA ACENTUADA
```

## Aplicações

Este sistema pode ser utilizado para:

- **Monitoramento de veículos**: Detecção de acidentes, freadas bruscas e curvas perigosas
- **Análise de movimento**: Estudos de biomecânica e análise de gestos
- **Sistemas de segurança**: Detecção de quedas e movimentos anômalos
- **Prototipagem IoT**: Base para sistemas de monitoramento wireless

## Limitações e Melhorias Futuras

### Limitações Atuais
- Alcance limitado do HC-12 (~1km em campo aberto)
- Processamento básico de eventos
- Sem armazenamento de dados
- Sem interface gráfica

### Melhorias Sugeridas
- Implementação de filtros digitais (Kalman, complementar)
- Adição de mais tipos de eventos detectados
- Interface web para visualização em tempo real
- Armazenamento de dados em SD card
- Calibração automática do sensor
- Modo de baixo consumo para bateria

## Troubleshooting

### Problemas Comuns

1. **Sensor não inicializa**
   - Verificar conexões I2C
   - Confirmar alimentação 3.3V
   - Testar endereço I2C

2. **HC-12 não comunica**
   - Verificar baudrate (9600)
   - Confirmar pinos TX/RX invertidos
   - Testar com comandos AT

3. **Dados inconsistentes**
   - Verificar estabilidade da alimentação
   - Calibrar sensor em posição estática
   - Ajustar filtros de ruído

## Licença

Este projeto foi desenvolvido para fins educacionais e de pesquisa.

## Contato

Para dúvidas ou sugestões sobre este projeto, entre em contato através dos canais de comunicação da disciplina.
