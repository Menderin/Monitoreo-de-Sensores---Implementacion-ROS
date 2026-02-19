#!/bin/bash

################################################################################
# Monitor de Microalgas UCN — Menú Principal
# 
# Punto de entrada único para gestionar todo el sistema.
# Orquesta: instalación, ESP32, credenciales y servicios Docker.
#
# Uso: ./menu.sh
################################################################################

set -e

# ============================================================================
# RUTAS
# ============================================================================
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
INSTALL_SH="$SCRIPT_DIR/install.sh"
MICROROS_SH="$SCRIPT_DIR/MicroROS - ESP/scripts/microros.sh"
ENV_MONGO="$SCRIPT_DIR/database/.env"
ENV_MONGO_EXAMPLE="$SCRIPT_DIR/.env.example"
ENV_WIFI="$SCRIPT_DIR/MicroROS - ESP/main/versions/wifi/.env"
ENV_WIFI_EXAMPLE="$SCRIPT_DIR/MicroROS - ESP/main/versions/wifi/.env.example"
SENSOR_NODE="$SCRIPT_DIR/database/ros_sensor_node.py"
MOTOR_NODE="$SCRIPT_DIR/MicroROS - ESP/main/Motores/motor_control_node.py"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

OS_TYPE="linux"
COMPOSE_CMD="docker compose"

# ============================================================================
# COLORES
# ============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

# ============================================================================
# UTILIDADES
# ============================================================================
success()  { echo -e "${GREEN}✓${NC} $1"; }
error_msg(){ echo -e "${RED}✗${NC} $1"; }
warning()  { echo -e "${YELLOW}⚠${NC} $1"; }
info()     { echo -e "${CYAN}ℹ${NC} $1"; }

prompt_continue() {
    echo ""
    read -rp "Presiona Enter para continuar..." _dummy
}

# Inicializa entorno ROS 2 limpio con venv dedicado al repo
# - Evita conflicto con conda (Python 3.13 vs ROS 3.12)
# - Las dependencias (pymongo, dotenv) quedan aisladas en .venv/
setup_ros_env() {
    local VENV_DIR="$SCRIPT_DIR/.venv"

    if [[ ! -f "$ROS_SETUP" ]]; then
        error_msg "ROS 2 Jazzy no encontrado en $ROS_SETUP"
        info "Instala ROS 2 Jazzy o usa la opción 1 (Docker)."
        return 1
    fi

    # ── 1. Limpiar conda del PATH para usar Python 3.12 del sistema ──────────
    if [[ -n "${CONDA_PREFIX:-}" ]]; then
        warning "Conda activo (${CONDA_DEFAULT_ENV:-base}) — limpiando para ROS 2 (Python 3.12)..."
        export PATH=$(echo "$PATH" | tr ':' '\n' \
            | grep -v "conda\|miniconda\|mamba" \
            | paste -sd ':')
        unset CONDA_DEFAULT_ENV CONDA_PREFIX CONDA_PYTHON_EXE CONDA_SHLVL
        unset PYTHONPATH
    fi

    # ── 2. Sourcear ROS 2 (añade sus paquetes a PYTHONPATH) ──────────────────
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

    # ── 3. Crear venv del repo si no existe ───────────────────────────────────
    # --system-site-packages permite que el venv vea rclpy y demás de ROS 2
    if [[ ! -d "$VENV_DIR" ]]; then
        info "Creando entorno virtual del repo en .venv/ ..."
        python3 -m venv --system-site-packages "$VENV_DIR"
        success "Entorno virtual creado"
    fi

    # ── 4. Activar venv ───────────────────────────────────────────────────────
    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"

    # ── 5. Instalar dependencias del repo si faltan ───────────────────────────
    local REQS=("pymongo>=4.0.0" "python-dotenv>=1.0.0" "certifi")
    local missing=()
    for pkg in "${REQS[@]}"; do
        pkg_name="${pkg%%[>=<]*}"
        python3 -c "import ${pkg_name//-/_}" 2>/dev/null || missing+=("$pkg")
    done

    if [[ ${#missing[@]} -gt 0 ]]; then
        info "Instalando dependencias en .venv: ${missing[*]}"
        pip install --quiet "${missing[@]}"
        success "Dependencias instaladas"
    fi

    success "Entorno listo — $(python3 --version 2>&1) | venv: .venv/"
}

# Estado de un archivo .env (OK / no configurado / ausente)
env_status() {
    local file="$1"
    if [[ ! -f "$file" ]]; then
        echo -e "${RED}[ausente]${NC}"
    elif grep -qE "^MONGO_URI=mongodb(\+srv)?://.+:.+@|^WIFI_SSID=.+" "$file" 2>/dev/null; then
        echo -e "${GREEN}[configurado]${NC}"
    else
        echo -e "${YELLOW}[sin configurar]${NC}"
    fi
}

# ============================================================================
# CABECERA
# ============================================================================
print_header() {
    clear
    echo -e "${BLUE}╔═══════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║                                                       ║${NC}"
    echo -e "${BLUE}║     ${CYAN}🌿 Monitor de Microalgas UCN — Control Center${BLUE}       ║${NC}"
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

# ============================================================================
# 1. INSTALACIÓN DEL SISTEMA
# ============================================================================
action_install() {
    print_section "1 — Instalación del sistema"

    if [[ ! -f "$INSTALL_SH" ]]; then
        error_msg "install.sh no encontrado en: $INSTALL_SH"
        prompt_continue; return
    fi

    info "Lanzando install.sh..."
    echo ""
    bash "$INSTALL_SH"
    prompt_continue
}

# ============================================================================
# 2. ACCIONES ESP32 (microros.sh)
# ============================================================================
action_esp32() {
    print_section "2 — Acciones ESP32"

    if [[ ! -f "$MICROROS_SH" ]]; then
        error_msg "microros.sh no encontrado en:"
        error_msg "  $MICROROS_SH"
        prompt_continue; return
    fi

    info "Abriendo menú ESP32 / micro-ROS..."
    echo ""
    # Ejecutar en subshell (al salir con 0 vuelve a este menú)
    bash "$MICROROS_SH" || true
}

# ============================================================================
# 3. CONFIGURAR CREDENCIALES
# ============================================================================

# 3a — MongoDB (.env raíz)
config_mongo() {
    print_section "3a — Credenciales MongoDB Atlas"

    if [[ ! -f "$ENV_MONGO" ]]; then
        if [[ -f "$ENV_MONGO_EXAMPLE" ]]; then
            cp "$ENV_MONGO_EXAMPLE" "$ENV_MONGO"
            success "Creado $ENV_MONGO desde plantilla"
        else
            # Crear archivo minimal
            cat > "$ENV_MONGO" <<'EOF'
MONGO_URI=
MONGO_DB=Datos_ESP
MONGO_COLLECTION=sensors_data
MONGO_COLLECTION_DISPOSITIVOS=devices_data
EOF
            success "Creado $ENV_MONGO vacío"
        fi
    fi

    info "Editando: $ENV_MONGO"
    echo ""
    echo -e "${YELLOW}  Variables clave:${NC}"
    echo "    MONGO_URI   — cadena de conexión Atlas (mongodb+srv://...)"
    echo "    MONGO_DB    — nombre de la base de datos"
    echo ""
    "${EDITOR:-nano}" "$ENV_MONGO"
    echo ""
    success "Archivo guardado."
    prompt_continue
}

# 3b — WiFi / Agent (.env del ESP32)
config_wifi() {
    print_section "3b — Credenciales WiFi y Agent (ESP32)"

    local wifi_dir
    wifi_dir="$(dirname "$ENV_WIFI")"

    if [[ ! -f "$ENV_WIFI" ]]; then
        if [[ -f "$ENV_WIFI_EXAMPLE" ]]; then
            cp "$ENV_WIFI_EXAMPLE" "$ENV_WIFI"
            success "Creado $ENV_WIFI desde plantilla"
        else
            mkdir -p "$wifi_dir"
            cat > "$ENV_WIFI" <<'EOF'
WIFI_SSID=
WIFI_PASSWORD=
AGENT_IP=
AGENT_PORT=8888
EOF
            success "Creado $ENV_WIFI vacío"
        fi
    fi

    # Mostrar IP actual como referencia
    echo ""
    info "IPs actuales de este PC (para AGENT_IP):"
    ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' \
        | while read -r ip; do echo "    📡 $ip"; done
    echo ""
    echo -e "${YELLOW}  Variables clave:${NC}"
    echo "    WIFI_SSID      — nombre de tu red WiFi"
    echo "    WIFI_PASSWORD  — contraseña de la red"
    echo "    AGENT_IP       — IP de este PC en la red"
    echo "    AGENT_PORT     — puerto UDP del Agent (default 8888)"
    echo ""
    info "Editando: $ENV_WIFI"
    echo ""
    "${EDITOR:-nano}" "$ENV_WIFI"
    echo ""
    success "Archivo guardado."
    info "El siguiente build (idf.py build) genera wifi_config.h automáticamente."
    prompt_continue
}

# Submenú de credenciales
action_credentials() {
    while true; do
        print_header
        echo -e "${YELLOW}╔═════════════════════════════════════════════════════════╗${NC}"
        echo -e "${YELLOW}║              3 — Configurar Credenciales                ║${NC}"
        echo -e "${YELLOW}╚═════════════════════════════════════════════════════════╝${NC}"
        echo ""
        echo -e "${CYAN}  🗄️  Base de datos${NC}"
        printf  "    a)  MongoDB Atlas (.env database/)      %s\n" "$(env_status "$ENV_MONGO")"
        echo ""
        echo -e "${CYAN}  📡  ESP32 / Firmware${NC}"
        printf  "    b)  WiFi + Agent IP (.env ESP32)         %s\n" "$(env_status "$ENV_WIFI")"
        echo ""
        echo -e "    ${RED}0)  Volver al menú principal${NC}"
        echo ""
        read -rp "Selecciona una opción: " sub
        echo ""
        case $sub in
            a|A) config_mongo ;;
            b|B) config_wifi ;;
            0)   break ;;
            *)   warning "Opción inválida"; sleep 1 ;;
        esac
    done
}

# ============================================================================
# 4. INICIAR NODO DE SENSORES
# ============================================================================
action_sensor_node() {
    print_section "4 — Nodo ROS 2: Sensores → MongoDB"

    if [[ ! -f "$SENSOR_NODE" ]]; then
        error_msg "ros_sensor_node.py no encontrado en:"
        error_msg "  $SENSOR_NODE"
        prompt_continue; return
    fi

    setup_ros_env || { prompt_continue; return; }

    info "Iniciando nodo de sensores... (Ctrl+C para detener)"
    echo ""
    python3 "$SENSOR_NODE" || true
    prompt_continue
}

# ============================================================================
# 5. INICIAR NODO DE MOTORES
# ============================================================================
action_motor_node() {
    print_section "5 — Nodo ROS 2: Control de Motores"

    if [[ ! -f "$MOTOR_NODE" ]]; then
        error_msg "motor_control_node.py no encontrado en:"
        error_msg "  $MOTOR_NODE"
        prompt_continue; return
    fi

    setup_ros_env || { prompt_continue; return; }

    info "Iniciando nodo de motores... (Ctrl+C para detener)"
    echo ""
    python3 "$MOTOR_NODE" || true
    prompt_continue
}

# ============================================================================
# 6. GESTIONAR SERVICIOS DOCKER
# ============================================================================
action_docker() {
    while true; do
        print_header
        echo -e "${YELLOW}╔═════════════════════════════════════════════════════════╗${NC}"
        echo -e "${YELLOW}║              6 — Servicios Docker                        ║${NC}"
        echo -e "${YELLOW}╚═════════════════════════════════════════════════════════╝${NC}"
        echo ""

        # Estado actual
        if command -v docker &>/dev/null; then
            local agent_status node_status
            agent_status=$(docker inspect -f '{{.State.Status}}' microros_agent 2>/dev/null || echo "no existe")
            node_status=$(docker inspect  -f '{{.State.Status}}' ros_sensor_node 2>/dev/null || echo "no existe")
            echo -e "  ${CYAN}Estado actual:${NC}"
            printf  "    microros_agent  : "
            [[ "$agent_status" == "running" ]] && echo -e "${GREEN}running${NC}" || echo -e "${YELLOW}$agent_status${NC}"
            printf  "    ros_sensor_node : "
            [[ "$node_status"  == "running" ]] && echo -e "${GREEN}running${NC}" || echo -e "${YELLOW}$node_status${NC}"
            echo ""
        fi

        echo -e "${CYAN}  ► Operaciones${NC}"
        echo "    a)  Iniciar servicios"
        echo "    b)  Detener servicios"
        echo "    c)  Reiniciar nodo de sensores   (aplicar cambios de código)"
        echo "    d)  Reconstruir imagen y reiniciar  (aplicar cambios de Dockerfile)"
        echo ""
        echo -e "${CYAN}  📜 Logs${NC}"
        echo "    e)  Logs en vivo — nodo de sensores"
        echo "    f)  Logs en vivo — micro-ROS Agent"
        echo "    g)  Estado general (docker compose ps)"
        echo ""
        echo -e "    ${RED}0)  Volver al menú principal${NC}"
        echo ""
        read -rp "Selecciona una opción: " sub
        echo ""

        case $sub in
            a|A)
                info "Iniciando servicios..."
                $COMPOSE_CMD up -d
                success "Servicios iniciados."
                prompt_continue ;;
            b|B)
                info "Deteniendo servicios..."
                $COMPOSE_CMD down
                success "Servicios detenidos."
                prompt_continue ;;
            c|C)
                info "Reiniciando nodo de sensores (sin rebuild)..."
                $COMPOSE_CMD restart ros_node
                success "Nodo reiniciado. El nuevo código está activo."
                prompt_continue ;;
            d|D)
                info "Reconstruyendo imagen y reiniciando servicios..."
                info "Esto puede tardar varios minutos..."
                $COMPOSE_CMD up -d --build
                success "Imagen reconstruida y servicios actualizados."
                prompt_continue ;;
            e|E)
                info "Mostrando logs de ros_sensor_node (Ctrl+C para salir)..."
                echo ""
                $COMPOSE_CMD logs -f ros_node || true
                prompt_continue ;;
            f|F)
                info "Mostrando logs de microros_agent (Ctrl+C para salir)..."
                echo ""
                $COMPOSE_CMD logs -f microros_agent || true
                prompt_continue ;;
            g|G)
                echo ""
                $COMPOSE_CMD ps
                prompt_continue ;;
            0) break ;;
            *) warning "Opción inválida"; sleep 1 ;;
        esac
    done
}

# ============================================================================
# ESTADO RÁPIDO (resumen en el menú principal)
# ============================================================================
show_status_bar() {
    local docker_status
    if command -v docker &>/dev/null && $COMPOSE_CMD ps --format json 2>/dev/null | grep -q '"Running"'; then
        docker_status="${GREEN}en ejecución${NC}"
    elif command -v docker &>/dev/null; then
        docker_status="${YELLOW}detenido${NC}"
    else
        docker_status="${RED}Docker no instalado${NC}"
    fi

    echo -e "  ${CYAN}Estado:${NC}"
    printf  "    Entorno        : %b\n" "${GREEN}Ubuntu Linux${NC}"
    printf  "    MongoDB .env   : %s\n" "$(env_status "$ENV_MONGO")"
    printf  "    WiFi .env      : %s\n" "$(env_status "$ENV_WIFI")"
    echo -e "    Servicios Docker: $docker_status"
    echo ""
}

# ============================================================================
# MENÚ PRINCIPAL
# ============================================================================
show_menu() {
    while true; do
        print_header
        show_status_bar

        echo -e "${YELLOW}╔═════════════════════════════════════════════════════════╗${NC}"
        echo -e "${YELLOW}║                    MENÚ PRINCIPAL                       ║${NC}"
        echo -e "${YELLOW}╚═════════════════════════════════════════════════════════╝${NC}"
        echo ""
        echo -e "${CYAN}  🖥️  Sistema${NC}"
        echo "    1)  Instalar sistema"
        echo "        (Docker, dependencias, clonar repo, iniciar servicios)"
        echo ""
        echo -e "${CYAN}  🔌  Hardware${NC}"
        echo "    2)  Acciones ESP32"
        echo "        (compilar, flashear, monitor serial, micro-ROS Agent)"
        echo ""
        echo -e "${CYAN}  🔑  Configuración${NC}"
        echo "    3)  Configurar credenciales"
        echo "        (MongoDB Atlas, WiFi, IP del Agent)"
        echo ""
        echo -e "${CYAN}  🤖  Nodos ROS 2${NC}"
        echo "    4)  Iniciar nodo de sensores"
        echo "        (ros_sensor_node.py → lee /sensor_data y guarda en MongoDB)"
        echo ""
        echo "    5)  Iniciar nodo de motores"
        echo "        (motor_control_node.py → control de actuadores)"
        echo ""
        echo -e "${CYAN}  🐳  Docker${NC}"
        echo "    6)  Gestionar servicios Docker"
        echo "        (iniciar, detener, reiniciar, logs, rebuild)"
        echo ""
        echo -e "    ${RED}7)  Salir${NC}"
        echo ""
        read -rp "Selecciona una opción [1-7]: " option
        echo ""

        case $option in
            1) action_install ;;
            2) action_esp32 ;;
            3) action_credentials ;;
            4) action_sensor_node ;;
            5) action_motor_node ;;
            6) action_docker ;;
            7)
                echo -e "${GREEN}¡Hasta luego!${NC}"
                exit 0
                ;;
            *)
                warning "Opción inválida"
                sleep 1
                ;;
        esac
    done
}

# ============================================================================
# ENTRADA
# ============================================================================
show_menu
