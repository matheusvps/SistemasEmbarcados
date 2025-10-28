# Sistema de Comunicação Binária - Lab2

## Resumo das Mudanças

O sistema foi modificado para enviar dados em formato binário ao invés de strings ASCII, com identificação única e verificação de integridade.

## Estrutura do Protocolo

### Pacote de Dados (`mpu_data_packet_t`)
```c
typedef struct {
    uint16_t magic;         // 0xAA55 - Identificador único
    int16_t ax, ay, az;    // Dados do acelerômetro
    int16_t gx, gy, gz;    // Dados do giroscópio
    uint8_t checksum;      // Checksum XOR para verificação
} mpu_data_packet_t;
```

**Tamanho total:** 15 bytes (vs ~20-30 bytes em ASCII)

## Principais Melhorias

### 1. Identificação Única
- **Magic Number:** `0xAA55` identifica pacotes válidos
- **Checksum:** Verificação XOR para integridade dos dados
- **Rejeição automática** de dados corrompidos ou inválidos

### 2. Eficiência de Transmissão
- **Formato binário:** Reduz tamanho dos pacotes em ~50%
- **Sem parsing:** Elimina necessidade de conversão string→número
- **Transmissão mais rápida** e confiável

### 3. Código Mais Legível
- **Constantes nomeadas:** Substitui valores hardcoded
- **Funções específicas:** Separação clara de responsabilidades
- **Comentários descritivos:** Melhor documentação

### 4. Configuração Centralizada
```c
// Constantes de configuração
#define SERIAL_BAUDRATE       115200
#define LED_BLINK_DELAY_MS     20
#define IMPACT_THRESHOLD      -1.2f
#define BRAKE_THRESHOLD       -0.8f
#define TURN_THRESHOLD        200.0f
```

## Fluxo de Comunicação

### Transmissor (STM32 - main.c)
1. Lê dados do MPU6050
2. Cria pacote binário com `HC12_CreateDataPacket()`
3. Envia pacote com `HC12_SendDataPacket()`
4. LED pisca como heartbeat

### Receptor (ESP32 - main.ino)
1. Recebe pacote com `receiveDataPacket()`
2. Verifica magic number e checksum
3. Processa dados válidos com `processSensorData()`
4. Detecta eventos (batida, freiada, curva)

## Funções Principais

### STM32 (Transmissor)
- `HC12_CreateDataPacket()` - Cria pacote com dados do sensor
- `HC12_SendDataPacket()` - Envia pacote binário
- `HC12_CalculateChecksum()` - Calcula verificação de integridade

### ESP32 (Receptor)
- `receiveDataPacket()` - Recebe e valida pacote
- `calculateChecksum()` - Verifica integridade
- `processSensorData()` - Processa dados e detecta eventos

## Benefícios

1. **Confiabilidade:** Verificação automática de integridade
2. **Eficiência:** Menor overhead de transmissão
3. **Robustez:** Rejeição de dados corrompidos
4. **Manutenibilidade:** Código mais organizado e legível
5. **Performance:** Processamento mais rápido dos dados

## Compatibilidade

- Mantém mesma interface de detecção de eventos
- Mesmos thresholds para batida, freiada e curva
- Saída serial idêntica para debugging
