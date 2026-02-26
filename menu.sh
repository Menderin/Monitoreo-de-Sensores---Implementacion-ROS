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
WIFI_ENV="$REPO_DIR/microros-esp/main/versions/wifi/.env"
SENSOR_NODE="$REPO_DIR/database/ros_sensor_node.py"
MOTOR_NODE="$REPO_DIR/microros-esp/main/Motores/motor_control_node.py"
MICROROS_WS="${MICROROS_WS:-$HOME/microros_ws}"
SCRIPTS_DIR="$REPO_DIR/scripts"
SERVICES_DIR="$REPO_DIR/services"

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

# ─── Source ESP-IDF ───────────────────────────────────────────────────────────
source_idf() {
    local _candidates=(
        "${IDF_PATH:-}/export.sh"
        "$HOME/esp/esp-idf/export.sh"
        "$HOME/esp/v5.5.2/esp-idf/export.sh"
        "$HOME/esp/v5.4.1/esp-idf/export.sh"
        "$HOME/esp/v5.3.2/esp-idf/export.sh"
    )
    for _f in "${_candidates[@]}"; do
        if [[ -n "$_f" && -f "$_f" ]]; then
            set +u; source "$_f" > /dev/null 2>&1; set -u
            success "ESP-IDF cargado desde $(dirname "$_f")"
            return 0
        fi
    done
    warn "ESP-IDF no encontrado. Algunas funciones de flash pueden fallar."
    warn "Ejecuta ./install.sh para instalarlo."
    return 1
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
    echo -e "  Archivo: ${YELLOW}microros-esp/main/versions/wifi/.env${RESET}"
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
        warn ".env de WiFi no encontrado. Creando plantilla mínima..."
        mkdir -p "$(dirname "$WIFI_ENV")"
        cat > "$WIFI_ENV" <<EOF
WIFI_SSID=TU_RED_WIFI
WIFI_PASSWORD=TU_CONTRASEÑA
AGENT_IP=${current_ip:-192.168.1.100}
AGENT_PORT=8888
EOF
        success "Creado: microros-esp/main/versions/wifi/.env"
    fi

    ${EDITOR:-nano} "$WIFI_ENV"
    echo ""
    success "Credenciales WiFi guardadas en microros-esp/main/versions/wifi/.env"
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

    local port
    port=$(get_agent_port)

    if pkill -f "micro_ros_agent" 2>/dev/null; then
        info "Procesos micro_ros_agent anteriores terminados."
        sleep 1
    fi

    info "Puerto: $port  |  Detener: Ctrl + C"
    echo ""
    bash --norc --noprofile -c "
        source /opt/ros/jazzy/setup.bash
        source '$MICROROS_WS/install/setup.bash'
        /opt/ros/jazzy/bin/ros2 run micro_ros_agent micro_ros_agent udp4 --port $port
    " || true

    echo ""
    info "Agent detenido."
    read -rp "  Presiona Enter para continuar..." _
}

# ─── Helper: ejecutar script Python con entorno ROS limpio (sin conda) ─────────
run_ros_python() {
    local script="$1"
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
    info "Activando entorno ESP-IDF (sin ROS para evitar conflictos en build)..."
    source_idf
    "$script"
}

# ==============================================================================
# 4. GATEWAY IOT — funciones
# ==============================================================================

# ── 4a. Ver estado del sistema ─────────────────────────────────────────────────
gateway_status() {
    echo ""
    local status_script="$SCRIPTS_DIR/status.sh"
    if [[ ! -f "$status_script" ]]; then
        error "scripts/status.sh no encontrado en $SCRIPTS_DIR"
        error "Asegúrate de que el repositorio esté completo."
        read -rp "  Presiona Enter para volver..." _
        return
    fi
    bash "$status_script"
    echo ""
    read -rp "  Presiona Enter para volver..." _
}

# ── 4b. Instalar / actualizar servicios systemd (One-Click) ───────────────────
deploy_services() {
    echo ""
    echo -e "${BOLD}  Instalar servicios systemd — One-Click Deploy${RESET}"
    echo ""

    # ── Verificación 1: escritura en el sistema de archivos ──────────────────
    info "Verificando escritura en el sistema de archivos..."
    local test_file
    test_file=$(mktemp 2>/dev/null) || {
        error "El sistema de archivos está en modo solo-lectura (¿SD dañada?)."
        error "Intenta remontarlo: sudo mount -o remount,rw /"
        read -rp "  Presiona Enter para volver..." _
        return 1
    }
    rm -f "$test_file"
    success "Sistema de archivos disponible para escritura."

    # ── Verificación 2: dependencias necesarias ───────────────────────────────
    info "Verificando dependencias..."
    local missing=0

    if ! dpkg -l iptables-persistent &>/dev/null 2>&1; then
        warn "  [!] iptables-persistent NO instalado — el firewall no sobrevivirá reinicios."
        warn "      Instala con: sudo apt-get install -y iptables-persistent"
        missing=$((missing + 1))
    else
        success "  iptables-persistent: OK"
    fi

    if ! python3 -c "import dotenv" &>/dev/null 2>&1; then
        warn "  [!] python3-dotenv NO disponible — el bridge de MongoDB puede fallar."
        warn "      Instala con: pip3 install python-dotenv --break-system-packages"
        missing=$((missing + 1))
    else
        success "  python3-dotenv: OK"
    fi

    if [[ $missing -gt 0 ]]; then
        echo ""
        warn "$missing dependencia(s) faltante(s). ¿Continuar de todas formas? [s/N]"
        read -rp "  Respuesta: " ans
        [[ "$ans" =~ ^[sS]$ ]] || { info "Deploy cancelado."; read -rp "  Presiona Enter..." _; return; }
    fi

    # ── Dar permisos de ejecución a scripts ──────────────────────────────────
    info "Aplicando permisos de ejecución a scripts/..."
    chmod +x "$SCRIPTS_DIR"/*.sh
    success "Permisos aplicados."

    # ── Generar e instalar los .service desde plantillas ─────────────────────
    local current_user="$USER"
    info "Desplegando servicios como usuario: ${BOLD}$current_user${RESET}"
    info "Scripts en: ${BOLD}$SCRIPTS_DIR${RESET}"
    echo ""

    for tpl in "$SERVICES_DIR"/smart-*.service.tpl; do
        [[ -f "$tpl" ]] || continue
        local svc_name
        svc_name=$(basename "$tpl" .tpl)   # → smart-agent.service / smart-bridge.service
        local dest="/etc/systemd/system/$svc_name"

        info "Generando $svc_name ..."
        sed \
            -e "s|{{USER}}|${current_user}|g" \
            -e "s|{{SCRIPTS_DIR}}|${SCRIPTS_DIR}|g" \
            "$tpl" | sudo tee "$dest" > /dev/null

        sudo chmod 644 "$dest"
        success "  Instalado → $dest"
    done

    # ── Recargar systemd y habilitar servicios ────────────────────────────────
    echo ""
    info "Recargando daemon systemd..."
    sudo systemctl daemon-reload

    info "Habilitando e iniciando servicios..."
    sudo systemctl enable --now smart-agent.service smart-bridge.service

    # ── Alias de acceso rápido en ~/.bashrc ───────────────────────────────────
    local alias_line="alias status='bash \"${SCRIPTS_DIR}/status.sh\"'"
    if grep -qF "alias status=" "$HOME/.bashrc" 2>/dev/null; then
        info "Alias 'status' ya existe en ~/.bashrc — sin cambios."
    else
        echo "" >> "$HOME/.bashrc"
        echo "# Gateway Monitoring System — acceso rápido al panel de estado" >> "$HOME/.bashrc"
        echo "$alias_line" >> "$HOME/.bashrc"
        success "Alias 'status' añadido a ~/.bashrc"
        warn "Abre una terminal nueva o ejecuta: source ~/.bashrc"
    fi

    # ── Post-Deploy Health Check ──────────────────────────────────────────────
    echo ""
    info "Esperando 5 segundos para que los servicios arranquen..."
    sleep 5

    local all_ok=true
    echo ""
    echo -e "${BOLD}  [ Post-Deploy Health Check ]${RESET}"

    for svc in smart-agent smart-bridge; do
        if systemctl is-active --quiet "$svc" 2>/dev/null; then
            success "  $svc: ${GREEN}ACTIVO ✓${RESET}"
        else
            error "  $svc: ${RED}CAÍDO ✗${RESET}"
            echo ""
            warn "  Últimas 5 líneas de log para $svc:"
            echo -e "${BLUE}  ──────────────────────────────────────${RESET}"
            sudo journalctl -u "$svc" -n 5 --no-pager 2>/dev/null \
                | sed 's/^/    /' || true
            echo -e "${BLUE}  ──────────────────────────────────────${RESET}"
            echo ""
            all_ok=false
        fi
    done

    echo ""
    if $all_ok; then
        success "════════════════════════════════════════════"
        success "  ¡Gateway activo y saludable!  🟢"
        success "════════════════════════════════════════════"
        echo ""
        warn "¿Abrir el panel de estado ahora para confirmar que todo quedó verde? [S/n]"
        read -rp "  Respuesta: " view_ans
        if [[ ! "$view_ans" =~ ^[nN]$ ]]; then
            bash "$SCRIPTS_DIR/status.sh"
            echo ""
        fi
    else
        warn "════════════════════════════════════════════"
        warn "  Uno o más servicios no arrancaron."
        warn "  Revisa los logs y ejecuta la opción d) para seguirlos en vivo."
        warn "════════════════════════════════════════════"
    fi

    read -rp "  Presiona Enter para continuar..." _
}

# ── 4c. Aplicar reglas de Firewall ────────────────────────────────────────────
apply_firewall() {
    echo ""
    echo -e "${BOLD}  Firewall — Modo Gateway Aislado${RESET}"
    echo ""
    echo -e "  ${YELLOW}Este script aplicará reglas iptables con la siguiente topología:${RESET}"
    echo -e "    ${CYAN}IF_WAN${RESET} (def: eth0)   → Internet / Red Lab  — SSH permitido"
    echo -e "    ${CYAN}IF_LAN${RESET} (def: wlan0)  → Red Sensores ESP32 — Solo DHCP, DNS, UDP (micro-ROS)"
    echo -e "    ${CYAN}FORWARD${RESET}              → DESACTIVADO (aislamiento)"
    echo ""
    echo -e "  ${YELLOW}Sobreescribir interfaces sin editar el script:${RESET}"
    echo -e "    export IF_WAN=eth1 IF_LAN=wlan1   (antes de esta opción)"
    echo ""

    local fw_script="$SCRIPTS_DIR/firewall.sh"
    if [[ ! -f "$fw_script" ]]; then
        error "scripts/firewall.sh no encontrado en $SCRIPTS_DIR"
        read -rp "  Presiona Enter para volver..." _
        return
    fi

    warn "¿Confirmas que las interfaces son correctas? [s/N]"
    read -rp "  Respuesta: " ans
    if [[ "$ans" =~ ^[sS]$ ]]; then
        sudo bash "$fw_script"
    else
        info "Firewall no aplicado."
        info "Edita scripts/firewall.sh o exporta IF_WAN/IF_LAN y vuelve a intentarlo."
    fi
    echo ""
    read -rp "  Presiona Enter para continuar..." _
}

# ── 4d. Seguir logs en vivo ───────────────────────────────────────────────────
follow_logs() {
    echo ""
    echo -e "${BOLD}  Logs en vivo — Gateway${RESET}"
    echo ""
    echo -e "  ${CYAN}a)${RESET}  smart-agent"
    echo -e "  ${CYAN}b)${RESET}  smart-bridge"
    echo -e "  ${CYAN}c)${RESET}  Ambos servicios"
    echo ""
    read -rp "  Opción: " lopt
    echo ""
    info "Detener: Ctrl + C"
    echo ""
    case "$lopt" in
        a|A) sudo journalctl -u smart-agent -f --no-pager || true ;;
        b|B) sudo journalctl -u smart-bridge -f --no-pager || true ;;
        c|C) sudo journalctl -u smart-agent -u smart-bridge -f --no-pager || true ;;
        *)   warn "Opción no válida." ;;
    esac
    echo ""
    read -rp "  Presiona Enter para continuar..." _
}

# ==============================================================================
# Submenús
# ==============================================================================

# ── Submenú: Credenciales ─────────────────────────────────────────────────────
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

# ── Submenú: Agentes (interactivos) ──────────────────────────────────────────
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

# ── Submenú: Gateway IoT ──────────────────────────────────────────────────────
menu_gateway() {
    while true; do
        show_header
        echo -e "  ${BOLD}4. Gateway IoT — Panel de control${RESET}"
        echo ""
        echo "    a)  Ver estado del sistema"
        echo "    b)  Instalar / actualizar servicios systemd"
        echo "    c)  Aplicar reglas de Firewall"
        echo "    d)  Seguir logs en vivo"
        echo ""
        echo "    0)  Volver"
        echo ""
        read -rp "  Opción: " opt
        case "$opt" in
            a|A) gateway_status ;;
            b|B) deploy_services ;;
            c|C) apply_firewall ;;
            d|D) follow_logs ;;
            0)   return ;;
            *)   warn "Opción no válida." ; sleep 1 ;;
        esac
    done
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
        echo "  4)  Gateway IoT — Panel de control"
        echo ""
        echo "  0)  Salir"
        echo ""
        read -rp "  Opción: " opt
        case "$opt" in
            1) menu_credentials ;;
            2) menu_agents ;;
            3) launch_esp32 ;;
            4) menu_gateway ;;
            0) echo "" ; info "Suerte con esas esp32!." ; echo "" ; exit 0 ;;
            *) warn "Opción no válida." ; sleep 1 ;;
        esac
    done
}

main_menu
