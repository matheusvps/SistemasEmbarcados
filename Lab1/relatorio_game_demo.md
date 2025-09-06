# Relatório - Game Demo STM32F411

## Objetivo
Implementar um game demo no microcontrolador STM32F411 com display ST7789, demonstrando as capacidades gráficas e de comunicação serial do sistema.

## Funcionalidades Implementadas

### 1. Bolinha Animada
- **Implementação**: Bolinha branca de raio 8 pixels que se move pela tela
- **Física**: Movimento com velocidade configurável (dx, dy) e colisão com bordas
- **Comportamento**: Quica nas bordas da tela e no cabeçalho (área de 30 pixels)
- **Observação**: O movimento é suave e responsivo, com detecção precisa de colisões

### 2. Cabeçalho com Uptime
- **Implementação**: Área de 30 pixels no topo da tela
- **Funcionalidade**: Exibe o tempo de funcionamento em segundos
- **Atualização**: Atualizado continuamente usando a função `millis()`
- **Observação**: O uptime é calculado corretamente e atualizado em tempo real

### 3. Quadrado Colorido
- **Implementação**: Quadrado de 20x20 pixels no canto superior direito
- **Funcionalidade**: Muda de cor a cada 1 segundo
- **Cores**: Ciclo entre 6 cores (vermelho, verde, azul, amarelo, ciano, magenta)
- **Observação**: A mudança de cor é precisa e sincronizada com o tempo

### 4. Log Serial
- **Implementação**: Impressão na serial a cada 200ms
- **Conteúdo**: Posição da bolinha (x, y) e uptime em milissegundos
- **Formato**: "Ball: x=120, y=120, Uptime: 1234 ms"
- **Observação**: O log é consistente e fornece informações úteis para debug

### 5. Controle de Velocidade
- **Implementação**: Comandos '+' e '-' via serial
- **Funcionalidade**: Aumenta/diminui a velocidade da bolinha
- **Limites**: Velocidade mínima de 1 pixel por frame
- **Observação**: O controle é responsivo e permite ajuste fino da velocidade

## Observações Técnicas

### Performance
- **FPS**: O jogo roda a aproximadamente 20 FPS (delay de 50ms)
- **Responsividade**: O sistema responde bem aos comandos seriais
- **Estabilidade**: Não foram observados travamentos ou comportamentos anômalos

### Uso de Recursos
- **CPU**: O loop principal é eficiente, com operações gráficas otimizadas
- **Memória**: Uso moderado de variáveis globais para estado do jogo
- **Display**: Uso eficiente das funções de desenho do ST7789

### Limitações Identificadas
- **Resolução**: O display de 240x240 pixels limita a complexidade visual
- **Velocidade**: O delay fixo de 50ms pode ser otimizado para diferentes velocidades
- **Cores**: Limitado ao conjunto de cores RGB565 predefinidas

## Conclusão

O game demo foi implementado com sucesso, demonstrando as capacidades do sistema STM32F411 com display ST7789. Todas as funcionalidades solicitadas foram implementadas e testadas, mostrando:

1. **Capacidade gráfica**: Desenho de formas geométricas e texto
2. **Animações**: Movimento suave e colisões precisas
3. **Comunicação serial**: Logs e controle em tempo real
4. **Timing**: Sincronização precisa de eventos temporais
5. **Interatividade**: Controle de velocidade via comandos seriais

O projeto demonstra a viabilidade de implementar aplicações gráficas interativas em sistemas embarcados com recursos limitados, utilizando técnicas eficientes de programação e otimização de recursos.

## Arquivos Modificados/Criados
- `src/game.c`: Implementação principal do jogo
- `src/game.h`: Cabeçalho com declarações
- `src/main.c`: Integração com o menu principal
