#!/bin/bash

################################################################################
# micro-ROS ESP32 - Script Todo-en-Uno
# 
# Script unificado para gestionar desarrollo con micro-ROS + ESP32 + ROS 2
# Incluye: build, flash, monitor, agent, instalación, diagnóstico
#
# Uso: ./microros.sh [comando]
#   Sin argumentos: Menú interactivo
#   Con comando: Ejecución directa
#
# Comandos disponibles:
#   build, flash, monitor, clean, menuconfig
#   agent-serial, agent-udp
#   topics, listen, node-info
#   install-agent, check-deps, fix-permissions
################################################################################

set -e  # Salir si hay error

# ============================================================================
# CONFIGURACIÓN
# ============================================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
ESP_IDF_PATH="/home/lab-ros/esp/v5.5.2/esp-idf"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
MICROROS_WS="$HOME/microros_ws"
ESP_PORT="/dev/ttyUSB0"
BAUDRATE="115200"

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# ============================================================================
# FUNCIONES AUXILIARES
# ============================================================================

print_header() {
    clear
    echo -e "${BLUE}╔═══════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║                                                       ║${NC}"
    echo -e "${BLUE}║      ${CYAN}🤖 micro-ROS ESP32 - Control Center 🚀${BLUE}      ║${NC}"
    echo -e "${BLUE}║                                                       ║${NC}"
    echo -e "${BLUE}╚═══════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_section() {
    echo ""
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${MAGENTA}  $1${NC}"
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

success() {
    echo -e "${GREEN}✓${NC} $1"
}

error() {
    echo -e "${RED}✗${NC} $1"
}

warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

info() {
    echo -e "${CYAN}ℹ${NC} $1"
}

prompt_continue() {
    echo ""
    read -p "Presiona Enter para continuar..." dummy
}

detect_esp_port() {
    if [ -e /dev/ttyUSB0 ]; then
        ESP_PORT="/dev/ttyUSB0"
    elif [ -e /dev/ttyUSB1 ]; then
        ESP_PORT="/dev/ttyUSB1"
    elif [ -e /dev/ttyACM0 ]; then
        ESP_PORT="/dev/ttyACM0"
    else
        warning "Puerto ESP32 no detectado automáticamente"
        read -p "Ingresa el puerto (ej: /dev/ttyUSB0): " ESP_PORT
    fi
    info "Puerto ESP32: $ESP_PORT"
}

# ============================================================================
# VERIFICACIÓN DE DEPENDENCIAS
# ============================================================================

check_dependencies() {
    print_section "Verificación de Dependencias"
    
    local all_ok=true
    
    # ROS 2
    if [ -f "$ROS_SETUP" ]; then
        success "ROS 2 Jazzy encontrado"
    else
        error "ROS 2 Jazzy NO encontrado en $ROS_SETUP"
        all_ok=false
    fi
    
    # ESP-IDF
    if [ -f "$ESP_IDF_PATH/export.sh" ]; then
        success "ESP-IDF encontrado"
    else
        error "ESP-IDF NO encontrado en $ESP_IDF_PATH"
        all_ok=false
    fi
    
    # Proyecto
    if [ -f "$PROJECT_DIR/CMakeLists.txt" ]; then
        success "Proyecto ESP32 encontrado"
    else
        error "CMakeLists.txt NO encontrado en $PROJECT_DIR"
        all_ok=false
    fi
    
    # micro-ROS Agent
    if command -v micro_ros_agent &> /dev/null || [ -d "$MICROROS_WS" ]; then
        success "micro-ROS Agent disponible"
    else
        warning "micro-ROS Agent NO encontrado (usa opción 'install-agent')"
    fi
    
    # Puerto USB
    if ls /dev/ttyUSB* /dev/ttyACM* &> /dev/null; then
        success "Puerto USB detectado: $(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n1)"
    else
        warning "No se detectó puerto USB (¿ESP32 conectado?)"
    fi
    
    # Permisos dialout
    if groups | grep -q dialout; then
        success "Usuario en grupo 'dialout'"
    else
        warning "Usuario NO está en grupo 'dialout' (ejecuta: fix-permissions)"
    fi
    
    echo ""
    if [ "$all_ok" = true ]; then
        success "Todas las dependencias críticas están OK"
        return 0
    else
        error "Faltan dependencias. Revisa los mensajes arriba."
        return 1
    fi
}

# ============================================================================
# FUNCIONES ESP32
# ============================================================================

source_esp_idf() {
    if [ ! -f "$ESP_IDF_PATH/export.sh" ]; then
        error "ESP-IDF no encontrado en $ESP_IDF_PATH"
        return 1
    fi
    
    info "Inicializando entorno ESP-IDF..."
    source "$ESP_IDF_PATH/export.sh" > /dev/null 2>&1
    success "Entorno ESP-IDF listo"
}

build_esp32() {
    print_section "Compilar Proyecto ESP32"
    
    cd "$PROJECT_DIR"
    source_esp_idf || return 1
    
    info "Compilando proyecto en: $PROJECT_DIR"
    idf.py build
    
    if [ $? -eq 0 ]; then
        success "Compilación exitosa"
        echo ""
        info "Binario generado: $PROJECT_DIR/build/hello_world.bin"
        return 0
    else
        error "Error en compilación"
        return 1
    fi
}

flash_esp32() {
    print_section "Flashear ESP32"
    
    cd "$PROJECT_DIR"
    source_esp_idf || return 1
    detect_esp_port
    
    # Verificar que el firmware esté compilado
    if [ ! -f "$PROJECT_DIR/build/hello_world.bin" ]; then
        warning "Firmware no compilado. Compilando primero..."
        build_esp32 || return 1
    fi
    
    info "Flasheando a puerto: $ESP_PORT"
    
    # Matar procesos que usan el puerto
    sudo fuser -k "$ESP_PORT" 2>/dev/null || true
    sleep 1
    
    idf.py -p "$ESP_PORT" flash
    
    if [ $? -eq 0 ]; then
        success "Flasheo exitoso"
        return 0
    else
        error "Error en flasheo"
        return 1
    fi
}

monitor_esp32() {
    print_section "Monitor Serial ESP32"
    
    cd "$PROJECT_DIR"
    source_esp_idf || return 1
    detect_esp_port
    
    info "Abriendo monitor en: $ESP_PORT @ $BAUDRATE"
    info "Salir: Ctrl + ]"
    echo ""
    sleep 2
    
    idf.py -p "$ESP_PORT" monitor
}

clean_esp32() {
    print_section "Limpiar Proyecto ESP32"
    
    cd "$PROJECT_DIR"
    source_esp_idf || return 1
    
    warning "Esto eliminará todos los archivos compilados"
    read -p "¿Continuar? (y/n): " confirm
    
    if [[ "$confirm" =~ ^[Yy]$ ]]; then
        info "Limpiando proyecto..."
        idf.py fullclean
        success "Proyecto limpiado"
        
        info "Recompilando desde cero..."
        build_esp32
    else
        info "Operación cancelada"
    fi
}

menuconfig_esp32() {
    print_section "Configuración ESP32 (menuconfig)"
    
    cd "$PROJECT_DIR"
    source_esp_idf || return 1
    
    info "Abriendo menuconfig..."
    idf.py menuconfig
}

build_flash_monitor() {
    print_section "Build + Flash + Monitor (Todo en Uno)"
    
    cd "$PROJECT_DIR"
    source_esp_idf || return 1
    detect_esp_port
    
    info "Ejecutando: build → flash → monitor"
    echo ""
    
    # Matar procesos del puerto
    sudo fuser -k "$ESP_PORT" 2>/dev/null || true
    sleep 1
    
    idf.py -p "$ESP_PORT" build flash monitor
}

erase_flash() {
    print_section "Borrar Flash Completa del ESP32"
    
    cd "$PROJECT_DIR"
    source_esp_idf || return 1
    detect_esp_port
    
    warning "Esto borrará TODA la flash del ESP32"
    read -p "¿Continuar? (y/n): " confirm
    
    if [[ "$confirm" =~ ^[Yy]$ ]]; then
        sudo fuser -k "$ESP_PORT" 2>/dev/null || true
        sleep 1
        
        info "Borrando flash..."
        idf.py -p "$ESP_PORT" erase-flash
        success "Flash borrada"
        
        warning "Ahora debes flashear el firmware de nuevo"
    else
        info "Operación cancelada"
    fi
}

# ============================================================================
# FUNCIONES ROS 2 / MICRO-ROS AGENT
# ============================================================================

source_ros2() {
    if [ ! -f "$ROS_SETUP" ]; then
        error "ROS 2 no encontrado en $ROS_SETUP"
        return 1
    fi
    
    source "$ROS_SETUP" > /dev/null 2>&1
    
    # Cargar workspace de micro-ROS si existe
    if [ -f "$MICROROS_WS/install/setup.bash" ]; then
        source "$MICROROS_WS/install/setup.bash" > /dev/null 2>&1
    fi
}

start_agent_serial() {
    print_section "Iniciar micro-ROS Agent (Serial)"
    
    source_ros2 || return 1
    detect_esp_port
    
    # Verificar que el agent esté instalado
    if ! command -v micro_ros_agent &> /dev/null && ! command -v ros2 &> /dev/null; then
        error "micro-ROS Agent no encontrado"
        warning "Ejecuta primero: ./microros.sh install-agent"
        return 1
    fi
    
    # Limpiar puerto - matar procesos que lo estén usando
    info "🧹 Limpiando puerto $ESP_PORT de conexiones previas..."
    if sudo fuser -k "$ESP_PORT" 2>/dev/null; then
        success "✓ Procesos previos terminados"
    else
        info "✓ Puerto limpio (no había procesos previos)"
    fi
    sleep 1
    
    info "Iniciando Agent en: $ESP_PORT @ $BAUDRATE"
    info "Detener: Ctrl + C"
    echo ""
    sleep 2
    
    # Intentar con ros2 run primero, luego con comando directo
    if command -v ros2 &> /dev/null; then
        ros2 run micro_ros_agent micro_ros_agent serial --dev "$ESP_PORT" -b "$BAUDRATE"
    else
        micro_ros_agent serial --dev "$ESP_PORT" -b "$BAUDRATE"
    fi
}

start_agent_udp() {
    print_section "Iniciar micro-ROS Agent (UDP/WiFi)"
    
    source_ros2 || return 1
    
    warning "NOTA: La versión actual del firmware usa SERIAL, no UDP"
    warning "Para usar UDP, debes reconfigurar y recompilar el ESP32"
    echo ""
    read -p "¿Continuar de todos modos? (y/n): " confirm
    
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        info "Operación cancelada"
        return 1
    fi
    
    # Limpiar cualquier proceso Agent previo
    info "🧹 Limpiando procesos Agent previos..."
    pkill -f "micro_ros_agent" 2>/dev/null && success "✓ Procesos Agent previos terminados" || info "✓ No había procesos Agent previos"
    sleep 1
    
    local port=8888
    read -p "Puerto UDP (Enter para 8888): " user_port
    [ -n "$user_port" ] && port=$user_port
    
    info "Iniciando Agent UDP en puerto: $port"
    info "Detener: Ctrl + C"
    echo ""
    sleep 2
    
    if command -v ros2 &> /dev/null; then
        ros2 run micro_ros_agent micro_ros_agent udp4 --port "$port"
    else
        micro_ros_agent udp4 --port "$port"
    fi
}

list_topics() {
    print_section "Listar Tópicos ROS 2"
    
    source_ros2 || return 1
    
    info "Tópicos disponibles:"
    echo ""
    ros2 topic list
    echo ""
    
    info "Para más detalles: ros2 topic info <topic_name>"
}

echo_temperature() {
    print_section "Escuchar Tópico /temperatura"
    
    source_ros2 || return 1
    
    if ! ros2 topic list 2>/dev/null | grep -q "/temperatura"; then
        warning "Tópico /temperatura no encontrado"
        warning "¿Está el ESP32 conectado y el Agent corriendo?"
        echo ""
        read -p "¿Ver todos los tópicos disponibles? (y/n): " show
        if [[ "$show" =~ ^[Yy]$ ]]; then
            list_topics
        fi
        return 1
    fi
    
    info "Escuchando /temperatura (Ctrl+C para salir)..."
    echo ""
    sleep 2
    
    ros2 topic echo /temperatura
}

node_info() {
    print_section "Información del Nodo ESP32"
    
    source_ros2 || return 1
    
    info "Nodos activos:"
    ros2 node list
    echo ""
    
    if ros2 node list 2>/dev/null | grep -q "esp32"; then
        success "Nodo 'esp32' encontrado"
        echo ""
        info "Información detallada:"
        ros2 node info /esp32
    else
        warning "Nodo 'esp32' no encontrado"
        warning "¿Está el ESP32 conectado y el Agent corriendo?"
    fi
}

topic_hz() {
    print_section "Frecuencia de Publicación"
    
    source_ros2 || return 1
    
    if ! ros2 topic list 2>/dev/null | grep -q "/temperatura"; then
        warning "Tópico /temperatura no encontrado"
        return 1
    fi
    
    info "Midiendo frecuencia de /temperatura..."
    info "Presiona Ctrl+C después de 10 segundos"
    echo ""
    sleep 2
    
    ros2 topic hz /temperatura
}

# ============================================================================
# INSTALACIÓN Y CONFIGURACIÓN
# ============================================================================

install_agent() {
    print_section "Instalar micro-ROS Agent"
    
    source_ros2 || return 1
    
    # Verificar si ya está instalado
    if command -v micro_ros_agent &> /dev/null; then
        success "micro-ROS Agent ya está instalado (snap)"
        warning "Si tienes problemas de permisos, instala desde fuente"
        echo ""
        read -p "¿Instalar desde fuente de todos modos? (y/n): " confirm
        if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
            return 0
        fi
    fi
    
    if [ -d "$MICROROS_WS" ]; then
        warning "Workspace $MICROROS_WS ya existe"
        read -p "¿Recompilar? (y/n): " recompile
        if [[ "$recompile" =~ ^[Yy]$ ]]; then
            cd "$MICROROS_WS"
            colcon build
            success "Agent recompilado"
            return 0
        fi
        return 0
    fi
    
    info "Instalando dependencias..."
    sudo apt update
    sudo apt install -y python3-pip ros-dev-tools
    
    info "Creando workspace en: $MICROROS_WS"
    mkdir -p "$MICROROS_WS/src"
    cd "$MICROROS_WS/src"
    
    info "Clonando repositorios..."
    git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git
    git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
    
    info "Compilando Agent (esto puede tardar varios minutos)..."
    cd "$MICROROS_WS"
    colcon build
    
    if [ $? -eq 0 ]; then
        success "micro-ROS Agent instalado correctamente"
        info "Ubicación: $MICROROS_WS"
        echo ""
        info "Para usar el Agent, se cargará automáticamente con este script"
        return 0
    else
        error "Error compilando el Agent"
        return 1
    fi
}

fix_permissions() {
    print_section "Configurar Permisos USB"
    
    info "Añadiendo usuario al grupo 'dialout'..."
    sudo usermod -a -G dialout "$USER"
    
    success "Usuario añadido al grupo 'dialout'"
    warning "Debes cerrar sesión y volver a iniciarla para aplicar los cambios"
    echo ""
    info "O ejecuta: newgrp dialout"
    echo ""
    
    if ls /dev/ttyUSB* /dev/ttyACM* &> /dev/null; then
        local port=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n1)
        info "Dando permisos temporales a: $port"
        sudo chmod 666 "$port"
        success "Permisos aplicados temporalmente"
    fi
}

# ============================================================================
# DIAGNÓSTICO Y UTILIDADES
# ============================================================================

list_serial_ports() {
    print_section "Puertos Seriales Disponibles"
    
    echo ""
    # Buscar puertos sin verificar permisos primero
    local ports=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null)
    
    if [ -n "$ports" ]; then
        success "Puertos encontrados:"
        for port in $ports; do
            if [ -r "$port" ] && [ -w "$port" ]; then
                echo "  ✓ $port (permisos OK)"
            else
                echo "  ✗ $port (sin permisos - ejecuta: ./microros.sh fix-permissions)"
            fi
        done
        echo ""
        info "Detalles:"
        ls -l $ports 2>/dev/null
    else
        warning "No se encontraron puertos USB"
        info "¿Está el ESP32 conectado?"
    fi
    
    echo ""
    info "Dispositivos USB:"
    lsusb | grep -i "CP210\|CH340\|FTDI\|Silicon\|Espressif" || echo "  (ninguno detectado)"
}

test_serial() {
    print_section "Test de Conexión Serial"
    
    detect_esp_port
    
    if [ ! -e "$ESP_PORT" ]; then
        error "Puerto $ESP_PORT no existe"
        return 1
    fi
    
    info "Verificando puerto: $ESP_PORT"
    ls -l "$ESP_PORT"
    echo ""
    
    info "Verificando permisos..."
    if [ -r "$ESP_PORT" ] && [ -w "$ESP_PORT" ]; then
        success "Permisos de lectura/escritura OK"
    else
        warning "Sin permisos suficientes"
        info "Ejecuta: ./microros.sh fix-permissions"
    fi
    
    echo ""
    info "Información del dispositivo:"
    udevadm info "$ESP_PORT" | grep -E "ID_VENDOR|ID_MODEL|ID_SERIAL" || true
}

show_system_info() {
    print_section "Información del Sistema"
    
    echo -e "${CYAN}Sistema Operativo:${NC}"
    cat /etc/os-release | grep -E "^NAME=|^VERSION=" || uname -a
    echo ""
    
    echo -e "${CYAN}ROS 2:${NC}"
    if [ -f "$ROS_SETUP" ]; then
        source "$ROS_SETUP" 2>/dev/null
        echo "  Distro: ${ROS_DISTRO:-desconocido}"
        echo "  Path: $ROS_SETUP"
    else
        echo "  No instalado"
    fi
    echo ""
    
    echo -e "${CYAN}ESP-IDF:${NC}"
    if [ -f "$ESP_IDF_PATH/version.txt" ]; then
        echo "  Versión: $(cat $ESP_IDF_PATH/version.txt)"
        echo "  Path: $ESP_IDF_PATH"
    else
        echo "  No encontrado"
    fi
    echo ""
    
    echo -e "${CYAN}micro-ROS Agent:${NC}"
    if command -v micro_ros_agent &> /dev/null; then
        echo "  Instalado (comando directo)"
    elif command -v ros2 &> /dev/null && ros2 pkg list | grep -q micro_ros_agent; then
        echo "  Instalado (ros2 package)"
    elif [ -d "$MICROROS_WS" ]; then
        echo "  Instalado (workspace: $MICROROS_WS)"
    else
        echo "  No instalado"
    fi
    echo ""
    
    echo -e "${CYAN}Proyecto:${NC}"
    echo "  Path: $PROJECT_DIR"
    if [ -f "$PROJECT_DIR/build/hello_world.bin" ]; then
        echo "  Firmware: Compilado ($(stat -c%s $PROJECT_DIR/build/hello_world.bin) bytes)"
    else
        echo "  Firmware: No compilado"
    fi
}

# ============================================================================
# MENÚ INTERACTIVO
# ============================================================================

show_menu() {
    print_header
    
    echo -e "${YELLOW}╔═════════════════════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║                    MENÚ PRINCIPAL                       ║${NC}"
    echo -e "${YELLOW}╚═════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    echo -e "${CYAN}  ESP32 - Desarrollo${NC}"
    echo "    1)  Compilar proyecto"
    echo "    2)  Flashear ESP32"
    echo "    3)  Monitor serial"
    echo "    4)  Build + Flash + Monitor (todo en uno)"
    echo "    5)  Limpiar proyecto (fullclean)"
    echo "    6)  Configuración (menuconfig)"
    echo "    7)  Borrar flash completa"
    echo ""
    
    echo -e "${CYAN}  micro-ROS Agent${NC}"
    echo "    8)  Iniciar Agent (Serial/UART)"
    echo "    9)  Iniciar Agent (UDP/WiFi)"
    echo ""
    
    echo -e "${CYAN}  ROS 2 - Monitoreo${NC}"
    echo "    10) Ver tópicos"
    echo "    11) Escuchar /temperatura"
    echo "    12) Info del nodo ESP32"
    echo "    13) Frecuencia de publicación (hz)"
    echo ""
    
    echo -e "${CYAN}  Instalación y Configuración${NC}"
    echo "    14) Instalar micro-ROS Agent"
    echo "    15) Verificar dependencias"
    echo "    16) Configurar permisos USB"
    echo ""
    
    echo -e "${CYAN}  Diagnóstico${NC}"
    echo "    17) Ver puertos seriales"
    echo "    18) Test conexión serial"
    echo "    19) Info del sistema"
    echo ""
    
    echo "    ${RED}0)  Salir${NC}"
    echo ""
    
    read -p "Selecciona una opción: " option
    echo ""
    
    case $option in
        1) build_esp32; prompt_continue; show_menu ;;
        2) flash_esp32; prompt_continue; show_menu ;;
        3) monitor_esp32; show_menu ;;
        4) build_flash_monitor; show_menu ;;
        5) clean_esp32; prompt_continue; show_menu ;;
        6) menuconfig_esp32; show_menu ;;
        7) erase_flash; prompt_continue; show_menu ;;
        8) start_agent_serial; show_menu ;;
        9) start_agent_udp; show_menu ;;
        10) list_topics; prompt_continue; show_menu ;;
        11) echo_temperature; show_menu ;;
        12) node_info; prompt_continue; show_menu ;;
        13) topic_hz; show_menu ;;
        14) install_agent; prompt_continue; show_menu ;;
        15) check_dependencies; prompt_continue; show_menu ;;
        16) fix_permissions; prompt_continue; show_menu ;;
        17) list_serial_ports; prompt_continue; show_menu ;;
        18) test_serial; prompt_continue; show_menu ;;
        19) show_system_info; prompt_continue; show_menu ;;
        0) echo -e "${GREEN}¡Hasta luego!${NC}"; exit 0 ;;
        *) error "Opción inválida"; sleep 2; show_menu ;;
    esac
}

# ============================================================================
# PROCESAMIENTO DE COMANDOS CLI
# ============================================================================

show_help() {
    cat << EOF
${BLUE}╔═══════════════════════════════════════════════════════════════╗${NC}
${BLUE}║      micro-ROS ESP32 - Script Todo-en-Uno v2.0               ║${NC}
${BLUE}╚═══════════════════════════════════════════════════════════════╝${NC}

${CYAN}USO:${NC}
  ./microros.sh                 Menú interactivo
  ./microros.sh [comando]       Ejecución directa

${CYAN}COMANDOS ESP32:${NC}
  build                         Compilar proyecto
  flash                         Flashear ESP32
  monitor                       Abrir monitor serial
  all                           Build + Flash + Monitor
  clean                         Limpiar proyecto
  menuconfig                    Abrir menuconfig
  erase-flash                   Borrar flash completa

${CYAN}COMANDOS MICRO-ROS AGENT:${NC}
  agent-serial                  Iniciar Agent por serial
  agent-udp                     Iniciar Agent por UDP/WiFi

${CYAN}COMANDOS ROS 2:${NC}
  topics                        Listar tópicos
  listen                        Escuchar /temperatura
  node-info                     Info del nodo ESP32
  hz                            Frecuencia de publicación

${CYAN}COMANDOS INSTALACIÓN:${NC}
  install-agent                 Instalar micro-ROS Agent
  check-deps                    Verificar dependencias
  fix-permissions               Configurar permisos USB

${CYAN}COMANDOS DIAGNÓSTICO:${NC}
  ports                         Ver puertos seriales
  test-serial                   Test conexión serial
  sysinfo                       Info del sistema

${CYAN}EJEMPLOS:${NC}
  ./microros.sh build           # Compilar
  ./microros.sh flash           # Flashear
  ./microros.sh all             # Todo en uno
  ./microros.sh agent-serial    # Iniciar Agent
  ./microros.sh listen          # Ver temperatura

EOF
}

# ============================================================================
# MAIN
# ============================================================================

main() {
    # Si no hay argumentos, mostrar menú interactivo
    if [ $# -eq 0 ]; then
        show_menu
        exit 0
    fi
    
    # Procesamiento de comandos CLI
    case "$1" in
        # ESP32
        build) build_esp32 ;;
        flash) flash_esp32 ;;
        monitor) monitor_esp32 ;;
        all|build-flash-monitor) build_flash_monitor ;;
        clean) clean_esp32 ;;
        menuconfig) menuconfig_esp32 ;;
        erase-flash) erase_flash ;;
        
        # Agent
        agent-serial|agent) start_agent_serial ;;
        agent-udp) start_agent_udp ;;
        
        # ROS 2
        topics|list-topics) list_topics ;;
        listen|echo|temperature) echo_temperature ;;
        node-info|node) node_info ;;
        hz|freq) topic_hz ;;
        
        # Instalación
        install-agent|install) install_agent ;;
        check-deps|check|verify) check_dependencies ;;
        fix-permissions|fix-perms|perms) fix_permissions ;;
        
        # Diagnóstico
        ports|list-ports) list_serial_ports ;;
        test-serial|test) test_serial ;;
        sysinfo|info) show_system_info ;;
        
        # Ayuda
        help|-h|--help) show_help ;;
        
        *)
            error "Comando desconocido: $1"
            echo ""
            info "Usa './microros.sh help' para ver comandos disponibles"
            info "O ejecuta sin argumentos para el menú interactivo"
            exit 1
            ;;
    esac
}

# Ejecutar
main "$@"
