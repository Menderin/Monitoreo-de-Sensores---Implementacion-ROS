#!/bin/bash

################################################################################
# micro-ROS ESP32 - Script Optimizado v2.1
# 
# Script mejorado con soporte WiFi y menú simplificado
# Incluye: build+flash, monitor, agent (serial/WiFi), diagnóstico
#
# Uso: ./microros.sh [comando]
#   Sin argumentos: Menú interactivo
#   Con comando: Ejecución directa
################################################################################

set -e  # Salir si hay error

# ============================================================================
# CONFIGURACIÓN
# ============================================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

# Auto-detect ESP-IDF path (busca en ubicaciones comunes)
_idf_candidates=(
    "${IDF_PATH:-}"
    "$HOME/esp/esp-idf"
    "$HOME/esp/v5.5.2/esp-idf"
    "$HOME/esp/v5.4.1/esp-idf"
    "$HOME/esp/v5.3.2/esp-idf"
)
ESP_IDF_PATH=""
for _d in "${_idf_candidates[@]}"; do
    if [[ -n "$_d" && -f "$_d/export.sh" ]]; then
        ESP_IDF_PATH="$_d"
        break
    fi
done
unset _idf_candidates _d
ROS_SETUP="/opt/ros/jazzy/setup.bash"
MICROROS_WS="$HOME/microros_ws"
ESP_PORT="/dev/ttyUSB0"
BAUDRATE="115200"
ENV_FILE="$PROJECT_DIR/main/.env"
AGENT_PORT=8888

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
    echo -e "${BLUE}║                  ${YELLOW}WiFi Enabled${BLUE}                  ║${NC}"
    echo -e "${BLUE}╚═══════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_section() {
    echo ""
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${MAGENTA}  $1${NC}"
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

success() { echo -e "${GREEN}✓${NC} $1"; }
error() { echo -e "${RED}✗${NC} $1"; }
warning() { echo -e "${YELLOW}⚠${NC} $1"; }
info() { echo -e "${CYAN}ℹ${NC} $1"; }

prompt_continue() {
    echo ""
    read -p "Presiona Enter para continuar..." dummy
}

detect_esp_port() {
    if [ -e /dev/ttyUSB0 ]; then ESP_PORT="/dev/ttyUSB0"
    elif [ -e /dev/ttyUSB1 ]; then ESP_PORT="/dev/ttyUSB1"
    elif [ -e /dev/ttyACM0 ]; then ESP_PORT="/dev/ttyACM0"
    else
        warning "Puerto ESP32 no detectado"
        read -p "Ingresa el puerto (ej: /dev/ttyUSB0): " ESP_PORT
    fi
    info "Puerto ESP32: $ESP_PORT"
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

    # Desactivar entorno ROS 2 para evitar conflictos con el colcon
    # interno de micro_ros_espidf_component durante el build
    unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX
    unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY
    unset COLCON_PREFIX_PATH ROS_PACKAGE_PATH
    unset CMAKE_PREFIX_PATH CMAKE_EXTENSION_PATH
    unset PYTHONPATH  # evita que catkin_pkg de ROS contamine el venv de IDF

    source "$ESP_IDF_PATH/export.sh"

    # Verificar que idf.py esté disponible
    if ! command -v idf.py &> /dev/null; then
        error "idf.py no disponible después de cargar ESP-IDF"
        error "Verifica la instalación de ESP-IDF en: $ESP_IDF_PATH"
        return 1
    fi

    success "Entorno ESP-IDF listo"
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

build_flash_monitor() {
    print_section "Build + Flash + Monitor (Todo en Uno)"
    cd "$PROJECT_DIR"
    source_esp_idf || return 1
    detect_esp_port

    # Generar sdkconfig.defaults desde .env para garantizar credenciales correctas
    local env_file="$PROJECT_DIR/main/versions/wifi/.env"
    if [[ -f "$env_file" ]]; then
        local ssid pass agent_ip agent_port
        ssid=$(grep "^WIFI_SSID=" "$env_file" | cut -d'=' -f2- | tr -d '[:space:]')
        pass=$(grep "^WIFI_PASSWORD=" "$env_file" | cut -d'=' -f2-)
        agent_ip=$(grep "^AGENT_IP=" "$env_file" | cut -d'=' -f2- | tr -d '[:space:]')
        agent_port=$(grep "^AGENT_PORT=" "$env_file" | cut -d'=' -f2- | tr -d '[:space:]')
        cat > "$PROJECT_DIR/sdkconfig.defaults" <<SDKEOF
CONFIG_ESP_WIFI_SSID="${ssid}"
CONFIG_ESP_WIFI_PASSWORD="${pass}"
CONFIG_MICRO_ROS_AGENT_IP="${agent_ip}"
CONFIG_MICRO_ROS_AGENT_PORT="${agent_port}"
CONFIG_ESP_WIFI_AUTH_WPA2_PSK=y
SDKEOF
        info "sdkconfig.defaults generado desde .env (SSID: ${ssid}, Agent: ${agent_ip}:${agent_port})"
    else
        warning "No se encontró .env en $env_file — usando credenciales del sdkconfig existente"
    fi

    info "Ejecutando: build → flash → monitor"
    echo ""

    # Matar procesos del puerto
    sudo fuser -k "$ESP_PORT" 2>/dev/null || true
    sleep 1

    idf.py -p "$ESP_PORT" -b 115200 build flash monitor
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

# ============================================================================
# FUNCIONES ROS 2 / MICRO-ROS AGENT
# ============================================================================

source_ros2() {
    if [ ! -f "$ROS_SETUP" ]; then
        error "ROS 2 no encontrado en $ROS_SETUP"
        return 1
    fi
    source "$ROS_SETUP" > /dev/null 2>&1
    
    # Source micro-ROS workspace si existe
    if [ -f "$MICROROS_WS/install/setup.bash" ]; then
        source "$MICROROS_WS/install/setup.bash" > /dev/null 2>&1
        success "micro-ROS Agent cargado desde workspace"
    else
        warning "Workspace micro-ROS no encontrado en $MICROROS_WS"
        info "Usa la opción 9 para instalar el Agent"
    fi
}

start_agent_serial() {
    print_section "Iniciar micro-ROS Agent (Serial)"
    source_ros2 || return 1
    detect_esp_port
    
    if ! command -v micro_ros_agent &> /dev/null && ! command -v ros2 &> /dev/null; then
        error "micro-ROS Agent no encontrado"
        warning "Ejecuta primero: ./microros.sh install-agent"
        return 1
    fi
    
    info "🧹 Limpiando puerto $ESP_PORT..."
    sudo fuser -k "$ESP_PORT" 2>/dev/null && success "Puerto limpio" || info "Puerto disponible"
    sleep 1
    
    info "Iniciando Agent en: $ESP_PORT @ $BAUDRATE"
    info "Detener: Ctrl + C"
    echo ""
    sleep 2
    
    if command -v ros2 &> /dev/null; then
        ros2 run micro_ros_agent micro_ros_agent serial --dev "$ESP_PORT" -b "$BAUDRATE"
    else
        micro_ros_agent serial --dev "$ESP_PORT" -b "$BAUDRATE"
    fi
}

start_agent_udp() {
    print_section "Iniciar micro-ROS Agent (UDP/WiFi)"
    source_ros2 || return 1
    
    # Leer puerto desde .env si existe
    if [ -f "$ENV_FILE" ]; then
        local env_port=$(grep "^AGENT_PORT=" "$ENV_FILE" | cut -d'=' -f2)
        [ -n "$env_port" ] && AGENT_PORT=$env_port
    fi
    
    info "🧹 Limpiando procesos Agent previos..."
    pkill -f "micro_ros_agent" 2>/dev/null && success "Procesos terminados" || info "No había procesos previos"
    sleep 1
    
    read -p "Puerto UDP (Enter para $AGENT_PORT): " user_port
    [ -n "$user_port" ] && AGENT_PORT=$user_port
    
    info "Iniciando Agent UDP en puerto: $AGENT_PORT"
    info "Detener: Ctrl + C"
    echo ""
    sleep 2
    
    if command -v ros2 &> /dev/null; then
        ros2 run micro_ros_agent micro_ros_agent udp4 --port "$AGENT_PORT"
    else
        micro_ros_agent udp4 --port "$AGENT_PORT"
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
# FUNCIONES WiFi
# ============================================================================

show_ip() {
    print_section "IP del PC"
    
    info "Direcciones IP del sistema:"
    echo ""
    ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | while read ip; do
        echo "  📡 $ip"
    done
    echo ""
    info "Usa esta IP en el archivo .env como AGENT_IP"
}

edit_env() {
    print_section "Editar Credenciales WiFi"
    
    if [ ! -f "$ENV_FILE" ]; then
        warning "Archivo .env no encontrado"
        info "Creando desde .env.example..."
        
        if [ -f "$PROJECT_DIR/.env.example" ]; then
            cp "$PROJECT_DIR/.env.example" "$ENV_FILE"
            success ".env creado desde plantilla"
        else
            error ".env.example no encontrado"
            return 1
        fi
    fi
    
    info "Abriendo editor..."
    ${EDITOR:-nano} "$ENV_FILE"
    
    success "Archivo guardado"
    info "Recuerda recompilar para aplicar cambios"
}

generate_wifi_config() {
    print_section "Generar wifi_config.h"
    
    if [ ! -f "$ENV_FILE" ]; then
        error "Archivo .env no encontrado"
        warning "Ejecuta primero: ./microros.sh edit-env"
        return 1
    fi
    
    info "Generando wifi_config.h desde .env..."
    python3 "$PROJECT_DIR/main/generate_wifi_config.py"
    
    if [ $? -eq 0 ]; then
        success "wifi_config.h generado correctamente"
        info "Ubicación: $PROJECT_DIR/main/wifi_config.h"
    else
        error "Error generando archivo"
        return 1
    fi
}

check_env() {
    print_section "Verificar Configuración .env"
    
    if [ ! -f "$ENV_FILE" ]; then
        error ".env no encontrado en: $ENV_FILE"
        info "Crea uno con: ./microros.sh edit-env"
        return 1
    fi
    
    success ".env encontrado"
    echo ""
    
    # Verificar variables requeridas
    local required=("WIFI_SSID" "WIFI_PASSWORD" "AGENT_IP" "AGENT_PORT")
    local missing=()
    
    for var in "${required[@]}"; do
        if grep -q "^${var}=" "$ENV_FILE"; then
            local value=$(grep "^${var}=" "$ENV_FILE" | cut -d'=' -f2)
            if [ -n "$value" ] && [ "$value" != "TU_RED_WIFI" ] && [ "$value" != "TU_CONTRASEÑA" ]; then
                success "$var configurado"
            else
                warning "$var está vacío o sin cambiar"
                missing+=("$var")
            fi
        else
            error "$var no encontrado"
            missing+=("$var")
        fi
    done
    
    echo ""
    if [ ${#missing[@]} -eq 0 ]; then
        success "Configuración completa ✓"
        return 0
    else
        warning "Faltan configurar: ${missing[*]}"
        info "Edita con: ./microros.sh edit-env"
        return 1
    fi
}

# ============================================================================
# INSTALACIÓN Y DIAGNÓSTICO
# ============================================================================

install_agent() {
    print_section "Instalar micro-ROS Agent"
    source_ros2 || return 1
    
    if command -v micro_ros_agent &> /dev/null; then
        success "micro-ROS Agent ya instalado"
        return 0
    fi
    
    if [ -d "$MICROROS_WS" ]; then
        warning "Workspace $MICROROS_WS ya existe"
        read -p "¿Recompilar? (y/n): " recompile
        if [[ "$recompile" =~ ^[Yy]$ ]]; then
            cd "$MICROROS_WS"
            colcon build
            success "Agent recompilado"
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
    
    info "Compilando Agent..."
    cd "$MICROROS_WS"
    colcon build
    
    if [ $? -eq 0 ]; then
        success "micro-ROS Agent instalado"
        return 0
    else
        error "Error compilando Agent"
        return 1
    fi
}

check_dependencies() {
    print_section "Verificación de Dependencias"
    
    local all_ok=true
    
    # ROS 2
    [ -f "$ROS_SETUP" ] && success "ROS 2 Jazzy" || { error "ROS 2 NO encontrado"; all_ok=false; }
    
    # ESP-IDF
    [ -f "$ESP_IDF_PATH/export.sh" ] && success "ESP-IDF" || { error "ESP-IDF NO encontrado"; all_ok=false; }
    
    # Proyecto
    [ -f "$PROJECT_DIR/CMakeLists.txt" ] && success "Proyecto ESP32" || { error "Proyecto NO encontrado"; all_ok=false; }
    
    # micro-ROS Agent
    if command -v micro_ros_agent &> /dev/null || [ -d "$MICROROS_WS" ]; then
        success "micro-ROS Agent"
    else
        warning "Agent NO encontrado (opción 9)"
    fi
    
    # .env
    [ -f "$ENV_FILE" ] && success "Archivo .env configurado" || warning ".env no encontrado (opción 12)"
    
    # Puerto USB
    ls /dev/ttyUSB* /dev/ttyACM* &> /dev/null && success "Puerto USB detectado" || warning "Puerto USB no detectado"
    
    # Permisos dialout
    groups | grep -q dialout && success "Permisos USB (dialout)" || warning "Sin permisos dialout"
    
    echo ""
    [ "$all_ok" = true ] && success "Dependencias críticas OK" || error "Faltan dependencias"
}

list_serial_ports() {
    print_section "Puertos Seriales"
    
    echo ""
    local ports=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null)
    
    if [ -n "$ports" ]; then
        success "Puertos encontrados:"
        for port in $ports; do
            [ -r "$port" ] && [ -w "$port" ] && echo "  ✓ $port (OK)" || echo "  ✗ $port (sin permisos)"
        done
        echo ""
        info "Detalles:"
        ls -l $ports 2>/dev/null
    else
        warning "No hay puertos USB"
        info "¿ESP32 conectado?"
    fi
    
    echo ""
    info "Dispositivos USB:"
    lsusb | grep -i "CP210\|CH340\|FTDI\|Silicon\|Espressif" || echo "  (ninguno detectado)"
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
    
    echo -e "${CYAN}  📡 ESP32 - Desarrollo${NC}"
    echo "    1)  Monitor serial"
    echo "    2)  Build + Flash + Monitor"
    echo "    3)  Limpiar proyecto (fullclean)"
    echo "    4)  Configuración (menuconfig)"
    echo ""
    
    echo -e "${CYAN}  🤖 micro-ROS Agent${NC}"
    echo "    5)  Agent (Serial/UART)"
    echo "    6)  Agent (UDP/WiFi)"
    echo ""
    
    echo -e "${CYAN}  📊 ROS 2 - Monitoreo${NC}"
    echo "    7)  Ver tópicos"
    echo "    8)  Frecuencia publicación (hz)"
    echo ""
    
    echo -e "${CYAN}  🔧 Instalación${NC}"
    echo "    9)  Instalar micro-ROS Agent"
    echo "    10) Verificar dependencias"
    echo ""
    
    echo -e "${CYAN}  📟 Diagnóstico${NC}"
    echo "    11) Ver puertos seriales"
    echo ""
    
    echo -e "${CYAN}  🌐 WiFi Config${NC}"
    echo "    12) Editar credenciales (.env)"
    echo "    13) Mostrar IP del PC"
    echo "    14) Generar wifi_config.h"
    echo "    15) Verificar configuración WiFi"
    echo ""
    
    echo -e "    ${RED}0)  Salir${NC}"
    echo ""
    
    read -rp "Selecciona una opción: " option
    echo ""
    
    case $option in
        1) monitor_esp32; show_menu ;;
        2) build_flash_monitor; show_menu ;;
        3) clean_esp32; prompt_continue; show_menu ;;
        4) menuconfig_esp32; show_menu ;;
        5) start_agent_serial; show_menu ;;
        6) start_agent_udp; show_menu ;;
        7) list_topics; prompt_continue; show_menu ;;
        8) topic_hz; show_menu ;;
        9) install_agent; prompt_continue; show_menu ;;
        10) check_dependencies; prompt_continue; show_menu ;;
        11) list_serial_ports; prompt_continue; show_menu ;;
        12) edit_env; prompt_continue; show_menu ;;
        13) show_ip; prompt_continue; show_menu ;;
        14) generate_wifi_config; prompt_continue; show_menu ;;
        15) check_env; prompt_continue; show_menu ;;
        0) echo -e "${GREEN}¡Hasta luego!${NC}"; exit 0 ;;
        *) error "Opción inválida"; sleep 2; show_menu ;;
    esac
}

# ============================================================================
# PROCESAMIENTO CLI
# ============================================================================

show_help() {
    cat << EOF
${BLUE}╔═══════════════════════════════════════════════════════════════╗${NC}
${BLUE}║      micro-ROS ESP32 - Script Optimizado v2.1                ║${NC}
${BLUE}╚═══════════════════════════════════════════════════════════════╝${NC}

${CYAN}USO:${NC}
  ./microros.sh                 Menú interactivo
  ./microros.sh [comando]       Ejecución directa

${CYAN}COMANDOS ESP32:${NC}
  monitor                       Monitor serial
  all                           Build + Flash + Monitor
  clean                         Limpiar proyecto
  menuconfig                    Configuración

${CYAN}COMANDOS AGENT:${NC}
  agent-serial                  Agent Serial/UART
  agent-udp                     Agent UDP/WiFi

${CYAN}COMANDOS ROS 2:${NC}
  topics                        Listar tópicos
  hz                            Frecuencia publicación

${CYAN}COMANDOS WiFi:${NC}
  edit-env                      Editar .env
  show-ip                       Ver IP del PC
  gen-wifi                      Generar wifi_config.h
  check-env                     Verificar .env

${CYAN}OTROS:${NC}
  install-agent                 Instalar Agent
  check-deps                    Verificar dependencias
  ports                         Ver puertos USB

EOF
}

main() {
    if [ $# -eq 0 ]; then
        show_menu
        exit 0
    fi
    
    case "$1" in
        monitor) monitor_esp32 ;;
        all|build-flash-monitor) build_flash_monitor ;;
        clean) clean_esp32 ;;
        menuconfig) menuconfig_esp32 ;;
        agent-serial|agent) start_agent_serial ;;
        agent-udp) start_agent_udp ;;
        topics) list_topics ;;
        hz) topic_hz ;;
        install-agent|install) install_agent ;;
        check-deps|check) check_dependencies ;;
        ports) list_serial_ports ;;
        edit-env) edit_env ;;
        show-ip) show_ip ;;
        gen-wifi) generate_wifi_config ;;
        check-env) check_env ;;
        help|-h|--help) show_help ;;
        *)
            error "Comando desconocido: $1"
            info "Usa './microros.sh help' para ver comandos disponibles"
            exit 1
            ;;
    esac
}

main "$@"
