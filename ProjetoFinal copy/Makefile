# Makefile para Projeto Final - Mini-Golf STM32F411
# Usa PlatformIO para compilação e upload

# Variáveis
ENV = blackpill_f411ce
PORT ?= COM3
BAUD = 115200

# Alvo padrão: limpa, compila e faz upload
all: clean build upload

# Compilar o projeto
build:
	@echo "Compilando projeto..."
	pio run -e $(ENV)

# Limpar arquivos de build
clean:
	@echo "Limpando arquivos de build..."
	pio run -e $(ENV) -t clean

# Fazer upload para o microcontrolador
upload:
	@echo "Fazendo upload para o microcontrolador..."
	pio run -e $(ENV) -t upload

# Compilar e fazer upload em uma única operação
build-upload: build upload

# Monitorar serial (com opção de especificar porta)
monitor:
	@echo "Monitorando serial (baudrate: $(BAUD))..."
	pio device monitor -e $(ENV) -b $(BAUD)

# Compilar, fazer upload e monitorar serial
deploy: build-upload
	@echo "Aguardando 2 segundos antes de iniciar monitor..."
	@sleep 2
	$(MAKE) monitor

# Verificar código (lint)
check:
	@echo "Verificando código..."
	pio check -e $(ENV)

# Mostrar informações do ambiente
info:
	@echo "Informações do ambiente:"
	pio run -e $(ENV) -t envdump

# Listar dispositivos conectados
devices:
	@echo "Dispositivos conectados:"
	pio device list

# Limpar tudo (incluindo dependências)
clean-all:
	@echo "Limpando tudo (incluindo dependências)..."
	pio run -e $(ENV) -t clean
	rm -rf .pio

# Ajuda
help:
	@echo "Makefile para Projeto Final - Mini-Golf STM32F411"
	@echo ""
	@echo "Comandos disponíveis:"
	@echo "  make              - Limpa, compila e faz upload (padrão)"
	@echo "  make build        - Apenas compila o projeto"
	@echo "  make clean        - Limpa arquivos de build"
	@echo "  make upload       - Apenas faz upload"
	@echo "  make build-upload - Compila e faz upload"
	@echo "  make monitor      - Monitora serial (115200 baud)"
	@echo "  make deploy       - Compila, faz upload e monitora serial"
	@echo "  make check        - Verifica código (lint)"
	@echo "  make info         - Mostra informações do ambiente"
	@echo "  make devices      - Lista dispositivos conectados"
	@echo "  make clean-all    - Limpa tudo (incluindo dependências)"
	@echo "  make help         - Mostra esta ajuda"
	@echo ""
	@echo "Variáveis:"
	@echo "  ENV=$(ENV)        - Ambiente PlatformIO"
	@echo "  PORT=$(PORT)      - Porta serial (use PORT=COM4 make monitor)"
	@echo "  BAUD=$(BAUD)      - Baudrate serial"

.PHONY: all build clean upload build-upload monitor deploy check info devices clean-all help

