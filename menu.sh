#!/usr/bin/env bash
# ==============================================================================
# menu.sh — Monitor de Microalgas UCN
# Menú principal del sistema
# ==============================================================================

set -euo pipefail

# ─── Rutas ────────────────────────────────────────────────────────────────────
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DB_ENV="$REPO_DIR/database/.env"
DB_ENV_EXAMPLE="$REPO_DIR/.env.example"
WIFI_ENV="$REPO_DIR/microros-esp/main/.env"
WIFI_ENV_TEMPLATE="$REPO_DIR/microros-esp/main/versions/wifi/.env"
SENSOR_NODE="$REPO_DIR/database/ros_sensor_node.py"
MOTOR_NODE="$REPO_DIR/microros-esp/main/Motores/motor_control_node.py"
MICROROS_WS="${MICROROS_WS:-$HOME/microros_ws}"

# ─── Colores ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BLUE='\033[0;34m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${CYAN}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}[ OK ]${RESET}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*"; }

# ─── Source ROS 2 ─────────────────────────────────────────────────────────────
source_ros() {
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        set +u; source /opt/ros/jazzy/setup.bash; set -u
    else
        error "ROS 2 Jazzy no encontrado. Ejecuta ./install.sh primero."
        return 1
    fi
    if [[ -f "$MICROROS_WS/install/setup.bash" ]]; then
        set +u; source "$MICROROS_WS/install/setup.bash"; set -u
    else
        warn "micro-ROS workspace no encontrado en $MICROROS_WS"
        warn "Ejecuta ./install.sh para instalarlo."
        return 1
    fi
}

# ─── Header ───────────────────────────────────────────────────────────────────
show_header() {
    clear
    echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════╗${RESET}"
    echo -e "${BOLD}${BLUE}║                                              ║${RESET}"
    echo -e "${BOLD}${BLUE}║   ${CYAN}Monitor de Microalgas UCN${BLUE}                 ║${RESET}"
    echo -e "${BOLD}${BLUE}║                                              ║${RESET}"
    echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════╝${RESET}"
    echo ""
}

# ==============================================================================
# 1a. Editar .env de base de datos
# ==============================================================================
edit_db_env() {
    echo ""
    echo -e "${BOLD}  Credenciales — Base de datos${RESET}"
    echo -e "  Archivo: ${YELLOW}database/.env${RESET}"
    echo ""

    if [[ ! -f "$DB_ENV" ]]; then
        warn ".env de base de datos no encontrado."
        if [[ -f "$DB_ENV_EXAMPLE" ]]; then
            info "Creando desde plantilla (.env.example)..."
            cp "$DB_ENV_EXAMPLE" "$DB_ENV"
            success "Creado: database/.env"
        else
            error ".env.example no encontrado. Crea database/.env manualmente."
            read -rp "  Presiona Enter para volver..." _
            return
        fi
    fi

    ${EDITOR:-nano} "$DB_ENV"
    echo ""
    success "Credenciales de base de datos guardadas."
    read -rp "  Presiona Enter para continuar..." _
}

# ==============================================================================
# 1b. Editar .env de WiFi
# ==============================================================================
edit_wifi_env() {
    echo ""
    echo -e "${BOLD}  Credenciales — WiFi / micro-ROS${RESET}"
    echo -e "  Archivo: ${YELLOW}microros-esp/main/.env${RESET}"
    echo ""
    echo -e "  Variables requeridas:"
    echo -e "    ${CYAN}WIFI_SSID${RESET}      — Nombre de la red WiFi"
    echo -e "    ${CYAN}WIFI_PASSWORD${RESET}  — Contraseña de la red"
    echo -e "    ${CYAN}AGENT_IP${RESET}       — IP de este PC en la red"
    echo -e "    ${CYAN}AGENT_PORT${RESET}     — Puerto UDP del Agent (default: 8888)"
    echo ""

    # Mostrar IP actual como referencia
    local current_ip
    current_ip=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | head -1 || true)
    if [[ -n "$current_ip" ]]; then
        info "IP actual de este PC: ${BOLD}$current_ip${RESET}"
    fi
    echo ""

    if [[ ! -f "$WIFI_ENV" ]]; then
        warn ".env de WiFi no encontrado."
        if [[ -f "$WIFI_ENV_TEMPLATE" ]]; then
            info "Creando desde plantilla..."
            cp "$WIFI_ENV_TEMPLATE" "$WIFI_ENV"
            success "Creado: microros-esp/main/.env"
        else
            # Crear plantilla mínima
            cat > "$WIFI_ENV" <<EOF
WIFI_SSID=TU_RED_WIFI
WIFI_PASSWORD=TU_CONTRASEÑA
AGENT_IP=${current_ip:-192.168.1.100}
AGENT_PORT=8888
EOF
            success "Creado: microros-esp/main/.env (plantilla mínima)"
        fi
    fi

    ${EDITOR:-nano} "$WIFI_ENV"
    echo ""
    success "Credenciales WiFi guardadas."
    warn "Recuerda recompilar el firmware del ESP32 para aplicar los cambios."
    read -rp "  Presiona Enter para continuar..." _
}

# ─── Helper: leer AGENT_PORT del .env WiFi ────────────────────────────────────
get_agent_port() {
    local port=8888
    if [[ -f "$WIFI_ENV" ]]; then
        local p
        p=$(grep "^AGENT_PORT=" "$WIFI_ENV" | cut -d'=' -f2 | tr -d '[:space:]' || true)
        [[ -n "$p" ]] && port=$p
    fi
    echo "$port"
}

# ==============================================================================
# 2a. Iniciar micro-ROS Agent (solo)
# ==============================================================================
start_agent_only() {
    echo ""
    echo -e "${BOLD}  Iniciar micro-ROS Agent (UDP)${RESET}"
    echo ""

    source_ros || { read -rp "  Presiona Enter para volver..." _; return; }

    local port
    port=$(get_agent_port)

    if pkill -f "micro_ros_agent" 2>/dev/null; then
        info "Procesos micro_ros_agent anteriores terminados."
        sleep 1
    fi

    info "Puerto: $port  |  Detener: Ctrl + C"
    echo ""
    ros2 run micro_ros_agent micro_ros_agent udp4 --port "$port" || true

    echo ""
    info "Agent detenido."
    read -rp "  Presiona Enter para continuar..." _
}

# ─── Helper: ejecutar script Python con entorno ROS limpio (sin conda) ─────────
# Uso: run_ros_python "/ruta/al/script.py"
run_ros_python() {
    local script="$1"
    # --norc --noprofile evita que conda inyecte su PYTHONPATH en el subshell
    bash --norc --noprofile -c "
        source /opt/ros/jazzy/setup.bash
        source '$MICROROS_WS/install/setup.bash'
        exec /usr/bin/python3 '$script'
    "
}

# ==============================================================================
# 2b. Enviar datos a MongoDB
# ==============================================================================
start_sensor_node() {
    echo ""
    echo -e "${BOLD}  Enviar datos a MongoDB${RESET}"
    echo ""
    info "Iniciando nodo ROS 2 → MongoDB..."
    info "Detener: Ctrl + C"
    echo ""

    run_ros_python "$SENSOR_NODE" || true

    echo ""
    info "Nodo terminado."
    read -rp "  Presiona Enter para continuar..." _
}

# ==============================================================================
# 2c. Control de motores
# ==============================================================================
start_motor_node() {
    echo ""
    echo -e "${BOLD}  Control de motores${RESET}"
    echo ""
    echo -e "  Controles:"
    echo -e "    ${CYAN}A/D${RESET}    — Izquierda / Derecha"
    echo -e "    ${CYAN}S${RESET}      — Detener"
    echo -e "    ${CYAN}1/2/3${RESET}  — Velocidad (40% / 70% / 100%)"
    echo -e "    ${CYAN}Q${RESET}      — Salir"
    echo ""
    info "Iniciando nodo de control de motores..."
    info "Detener: Q o Ctrl + C"
    echo ""

    run_ros_python "$MOTOR_NODE" || true

    echo ""
    info "Nodo terminado."
    read -rp "  Presiona Enter para continuar..." _
}

# ==============================================================================
# Submenú: Credenciales
# ==============================================================================
menu_credentials() {
    while true; do
        show_header
        echo -e "  ${BOLD}1. Modificar credenciales${RESET}"
        echo ""
        echo "    a)  Base de datos (database/.env)"
        echo "    b)  WiFi / micro-ROS (microros-esp/main/.env)"
        echo ""
        echo "    0)  Volver"
        echo ""
        read -rp "  Opción: " opt
        case "$opt" in
            a|A) edit_db_env ;;
            b|B) edit_wifi_env ;;
            0)   return ;;
            *)   warn "Opción no válida." ; sleep 1 ;;
        esac
    done
}

# ==============================================================================
# Submenú: Agentes
# ==============================================================================
menu_agents() {
    while true; do
        show_header
        echo -e "  ${BOLD}2. Iniciar agentes${RESET}"
        echo ""
        echo "    a)  Iniciar micro-ROS Agent (UDP)"
        echo "    b)  Enviar datos a MongoDB"
        echo "    c)  Control de motores"
        echo ""
        echo "    0)  Volver"
        echo ""
        read -rp "  Opción: " opt
        case "$opt" in
            a|A) start_agent_only ;;
            b|B) start_sensor_node ;;
            c|C) start_motor_node ;;
            0)   return ;;
            *)   warn "Opción no válida." ; sleep 1 ;;
        esac
    done
}

# ==============================================================================
# 3. ESP32 — lanzar microros.sh
# ==============================================================================
launch_esp32() {
    local script="$REPO_DIR/microros-esp/scripts/microros.sh"
    if [[ ! -x "$script" ]]; then
        if [[ -f "$script" ]]; then
            chmod +x "$script"
        else
            error "No se encontró $script"
            read -rp "  Presiona Enter para volver..." _
            return
        fi
    fi
    info "Activando entorno ROS 2 + micro-ROS..."
    source_ros || { read -rp "  Presiona Enter para volver..." _; return; }
    "$script"
}

# ==============================================================================
# Menú principal
# ==============================================================================
main_menu() {
    while true; do
        show_header
        echo "  1)  Modificar credenciales"
        echo "  2)  Iniciar agentes"
        echo "  3)  Desplegar opciones ESP32"
        echo ""
        echo "  0)  Salir"
        echo ""
        read -rp "  Opción: " opt
        case "$opt" in
            1) menu_credentials ;;
            2) menu_agents ;;
            3) launch_esp32 ;;
            0) echo "" ; info "Suerte con esas esp32!." ; echo "" ; exit 0 ;;
            *) warn "Opción no válida." ; sleep 1 ;;
        esac
    done
}

main_menu
