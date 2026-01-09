#!/bin/bash

# Script de ayuda para micro-ROS con ESP32
# Ubicación: /home/lab-ros/Documentos/Github/microRostest/

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ESP_IDF_PATH="/home/lab-ros/esp/v5.5.2/esp-idf"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

# Colores
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

function print_header() {
    echo -e "${BLUE}╔════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║   micro-ROS ESP32 - Herramienta de ayuda  ║${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════════╝${NC}"
    echo ""
}

function check_ros2() {
    if [ -f "$ROS_SETUP" ]; then
        echo -e "${GREEN}✓ ROS 2 Jazzy encontrado${NC}"
        return 0
    else
        echo -e "${RED}✗ ROS 2 Jazzy no encontrado en $ROS_SETUP${NC}"
        return 1
    fi
}

function check_esp_idf() {
    if [ -f "$ESP_IDF_PATH/export.sh" ]; then
        echo -e "${GREEN}✓ ESP-IDF encontrado${NC}"
        return 0
    else
        echo -e "${RED}✗ ESP-IDF no encontrado en $ESP_IDF_PATH${NC}"
        return 1
    fi
}

function menu() {
    clear
    print_header
    
    echo "Verificando dependencias..."
    check_ros2
    check_esp_idf
    echo ""
    
    echo "Selecciona una opción:"
    echo ""
    echo "  ${YELLOW}ESP32:${NC}"
    echo "    1) Compilar proyecto ESP32"
    echo "    2) Flashear ESP32"
    echo "    3) Monitor ESP32"
    echo "    4) Build + Flash + Monitor"
    echo "    5) Limpiar proyecto"
    echo ""
    echo "  ${YELLOW}ROS 2:${NC}"
    echo "    6) Iniciar micro-ROS Agent (Serial)"
    echo "    7) Iniciar micro-ROS Agent (UDP/WiFi)"
    echo "    8) Ver tópicos ROS"
    echo "    9) Escuchar temperatura"
    echo "   10) Ver información del nodo"
    echo ""
    echo "  ${YELLOW}Utilidades:${NC}"
    echo "   11) Ver puertos seriales disponibles"
    echo "   12) Dar permisos a puerto USB"
    echo "   13) Configurar WiFi (menuconfig)"
    echo "   14) Ver esta ayuda"
    echo ""
    echo "    0) Salir"
    echo ""
    read -p "Opción: " option
    
    case $option in
        1) build_esp32 ;;
        2) flash_esp32 ;;
        3) monitor_esp32 ;;
        4) build_flash_monitor ;;
        5) clean_esp32 ;;
        6) start_agent_serial ;;
        7) start_agent_udp ;;
        8) list_topics ;;
        9) echo_temperature ;;
        10) node_info ;;
        11) list_serial_ports ;;
        12) fix_usb_permissions ;;
        13) menuconfig_esp32 ;;
        14) show_help ;;
        0) exit 0 ;;
        *) echo -e "${RED}Opción inválida${NC}"; sleep 2; menu ;;
    esac
}

function source_esp_idf() {
    source "$ESP_IDF_PATH/export.sh" > /dev/null 2>&1
}

function build_esp32() {
    echo -e "${BLUE}🔨 Compilando proyecto...${NC}"
    cd "$SCRIPT_DIR"
    source_esp_idf
    idf.py build
    read -p "Presiona Enter para continuar..."
    menu
}

function flash_esp32() {
    echo -e "${BLUE}📤 Flasheando ESP32...${NC}"
    list_serial_ports
    read -p "Puerto (ej: /dev/ttyUSB0): " port
    cd "$SCRIPT_DIR"
    source_esp_idf
    idf.py -p "$port" flash
    read -p "Presiona Enter para continuar..."
    menu
}

function monitor_esp32() {
    echo -e "${BLUE}🖥️ Abriendo monitor (Ctrl+] para salir)...${NC}"
    sleep 2
    cd "$SCRIPT_DIR"
    source_esp_idf
    idf.py monitor
    menu
}

function build_flash_monitor() {
    echo -e "${BLUE}🚀 Build + Flash + Monitor...${NC}"
    list_serial_ports
    read -p "Puerto (ej: /dev/ttyUSB0): " port
    cd "$SCRIPT_DIR"
    source_esp_idf
    idf.py -p "$port" build flash monitor
    menu
}

function clean_esp32() {
    echo -e "${BLUE}🧹 Limpiando proyecto...${NC}"
    cd "$SCRIPT_DIR"
    source_esp_idf
    idf.py fullclean
    echo -e "${GREEN}✓ Proyecto limpio${NC}"
    read -p "Presiona Enter para continuar..."
    menu
}

function start_agent_serial() {
    echo -e "${BLUE}🔌 Iniciando micro-ROS Agent (Serial)...${NC}"
    list_serial_ports
    read -p "Puerto (ej: /dev/ttyUSB0): " port
    source "$ROS_SETUP"
    echo -e "${GREEN}Agente corriendo... Presiona Ctrl+C para detener${NC}"
    ros2 run micro_ros_agent micro_ros_agent serial --dev "$port" -b 115200
    menu
}

function start_agent_udp() {
    echo -e "${BLUE}📡 Iniciando micro-ROS Agent (UDP)...${NC}"
    read -p "Puerto UDP (default: 8888): " port
    port=${port:-8888}
    source "$ROS_SETUP"
    echo -e "${GREEN}Agente corriendo en puerto UDP $port... Presiona Ctrl+C para detener${NC}"
    ros2 run micro_ros_agent micro_ros_agent udp4 --port "$port"
    menu
}

function list_topics() {
    echo -e "${BLUE}📋 Tópicos ROS disponibles:${NC}"
    source "$ROS_SETUP"
    ros2 topic list
    echo ""
    read -p "Presiona Enter para continuar..."
    menu
}

function echo_temperature() {
    echo -e "${BLUE}🌡️ Escuchando temperatura (Ctrl+C para detener)...${NC}"
    sleep 2
    source "$ROS_SETUP"
    ros2 topic echo /temperatura
    menu
}

function node_info() {
    echo -e "${BLUE}ℹ️ Información del nodo:${NC}"
    source "$ROS_SETUP"
    echo ""
    echo "Nodos activos:"
    ros2 node list
    echo ""
    echo "Info del tópico /temperatura:"
    ros2 topic info /temperatura
    echo ""
    read -p "Presiona Enter para continuar..."
    menu
}

function list_serial_ports() {
    echo -e "${YELLOW}Puertos seriales disponibles:${NC}"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "No se encontraron puertos /dev/ttyUSB*"
    ls -la /dev/ttyACM* 2>/dev/null || echo "No se encontraron puertos /dev/ttyACM*"
    echo ""
}

function fix_usb_permissions() {
    echo -e "${BLUE}🔐 Configurando permisos USB...${NC}"
    list_serial_ports
    read -p "Puerto a configurar (ej: /dev/ttyUSB0): " port
    if [ -e "$port" ]; then
        sudo chmod 666 "$port"
        echo -e "${GREEN}✓ Permisos configurados para $port${NC}"
    else
        echo -e "${RED}✗ Puerto no encontrado${NC}"
    fi
    read -p "Presiona Enter para continuar..."
    menu
}

function menuconfig_esp32() {
    echo -e "${BLUE}⚙️ Abriendo menuconfig...${NC}"
    cd "$SCRIPT_DIR"
    source_esp_idf
    idf.py menuconfig
    menu
}

function show_help() {
    clear
    print_header
    echo -e "${GREEN}FLUJO DE TRABAJO TÍPICO:${NC}"
    echo ""
    echo "1. Conectar ESP32 por USB"
    echo "2. Compilar y flashear: Opción 4 (Build + Flash + Monitor)"
    echo "3. Verificar que el ESP32 funciona (ver temperaturas en monitor)"
    echo "4. Cerrar monitor (Ctrl+])"
    echo "5. Iniciar micro-ROS Agent: Opción 6 (Serial)"
    echo "6. En otra terminal, ver tópicos: Opción 8"
    echo "7. Escuchar temperatura: Opción 9"
    echo ""
    echo -e "${YELLOW}COMANDOS MANUALES:${NC}"
    echo ""
    echo "ESP-IDF:"
    echo "  source $ESP_IDF_PATH/export.sh"
    echo "  idf.py build flash monitor"
    echo ""
    echo "ROS 2:"
    echo "  source $ROS_SETUP"
    echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200"
    echo "  ros2 topic list"
    echo "  ros2 topic echo /temperatura"
    echo ""
    read -p "Presiona Enter para volver al menú..."
    menu
}

# Iniciar menú
menu
