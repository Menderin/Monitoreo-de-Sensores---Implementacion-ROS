#!/usr/bin/env bash
# ==============================================================================
# menu.sh — Monitor de Microalgas UCN  v3.0
# Menú principal del sistema — Raspberry Pi 400 / Gateway BioFloc
#
# Estructura:
#   0) Instalación de Dependencias   (ROS 2, ESP-IDF, micro-ROS)
#   1) Configuración del Sistema     (MongoDB, WiFi, Telegram, Hotspot)
#   2) Despliegue de Sensores        (Compilar y flashear ESP32)
#   3) Activación de Servicios       (Systemd, Cron, Firewall)
#   4) Diagnóstico y Operación       (Estado, Logs, Tópicos ROS)
# ==============================================================================

set -euo pipefail

# ─── Rutas ────────────────────────────────────────────────────────────────────
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DB_ENV="$REPO_DIR/database/.env"
DB_ENV_EXAMPLE="$REPO_DIR/.env.example"
WIFI_ENV="$REPO_DIR/microros-esp/main/versions/wifi/.env"
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
        error "ROS 2 Jazzy no encontrado. Ejecuta la opción 0 para instalar."
        return 1
    fi
    if [[ -f "$MICROROS_WS/install/setup.bash" ]]; then
        set +u; source "$MICROROS_WS/install/setup.bash"; set -u
    else
        warn "micro-ROS workspace no encontrado en $MICROROS_WS"
        warn "Ejecuta la opción 0 para instalarlo."
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
    warn "ESP-IDF no encontrado. Ejecuta la opción 0 para instalarlo."
    return 1
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

# ─── Helper: ejecutar Python con entorno ROS limpio ───────────────────────────
run_ros_python() {
    local script="$1"
    bash --norc --noprofile -c "
        source /opt/ros/jazzy/setup.bash
        source '$MICROROS_WS/install/setup.bash'
        exec /usr/bin/python3 '$script'
    "
}

# ─── Header ───────────────────────────────────────────────────────────────────
show_header() {
    clear
    echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════╗${RESET}"
    echo -e "${BOLD}${BLUE}║                                              ║${RESET}"
    echo -e "${BOLD}${BLUE}║   ${CYAN}Monitor de Microalgas UCN  v3.0${BLUE}          ║${RESET}"
    echo -e "${BOLD}${BLUE}║   ${YELLOW}Raspberry Pi 400 — Gateway BioFloc${BLUE}       ║${RESET}"
    echo -e "${BOLD}${BLUE}║                                              ║${RESET}"
    echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════╝${RESET}"
    echo ""
}

# ==============================================================================
# OPCIÓN 0 — Instalación de Dependencias
# ==============================================================================
run_installer() {
    echo ""
    echo -e "${BOLD}  0. Instalación de Dependencias${RESET}"
    echo ""

    # ── Estado actual del sistema ─────────────────────────────────────────────
    echo -e "  ${BOLD}Estado actual:${RESET}"
    echo ""

    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        success "  ROS 2 Jazzy      : INSTALADO"
    else
        warn    "  ROS 2 Jazzy      : NO instalado"
    fi

    local _idf_found=false
    for _d in "${IDF_PATH:-}" "$HOME/esp/esp-idf" "$HOME/esp/v5.5.2/esp-idf" "$HOME/esp/v5.4.1/esp-idf"; do
        if [[ -n "$_d" && -f "$_d/export.sh" ]]; then
            success "  ESP-IDF          : INSTALADO  ($_d)"
            _idf_found=true; break
        fi
    done
    $_idf_found || warn "  ESP-IDF          : NO instalado"

    if [[ -f "$MICROROS_WS/install/setup.bash" ]]; then
        success "  micro-ROS Agent  : INSTALADO  ($MICROROS_WS)"
    else
        warn    "  micro-ROS Agent  : NO instalado"
    fi

    echo ""
    warn "El instalador puede tardar 15–30 min (compilación en Raspberry Pi)."
    read -rp "  ¿Ejecutar ./install.sh ahora? [s/N]: " ans
    if [[ "$ans" =~ ^[sS]$ ]]; then
        echo ""
        info "Iniciando instalador..."
        echo ""
        bash "$REPO_DIR/install.sh"
        echo ""
        success "Instalador finalizado."
        info "Abre una terminal nueva o ejecuta: source ~/.bashrc"
    else
        info "Instalación cancelada."
    fi
    read -rp "  Presiona Enter para continuar..." _
}

# ==============================================================================
# OPCIÓN 1 — Configuración del Sistema
# ==============================================================================

# ── 1a. .env de base de datos ─────────────────────────────────────────────────
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

# ── 1b. .env de WiFi / micro-ROS ─────────────────────────────────────────────
edit_wifi_env() {
    echo ""
    echo -e "${BOLD}  Credenciales — WiFi / micro-ROS${RESET}"
    echo -e "  Archivo: ${YELLOW}microros-esp/main/versions/wifi/.env${RESET}"
    echo ""
    echo -e "  Variables requeridas:"
    echo -e "    ${CYAN}WIFI_SSID${RESET}      — Nombre de la red WiFi (o el Hotspot)"
    echo -e "    ${CYAN}WIFI_PASSWORD${RESET}  — Contraseña de la red"
    echo -e "    ${CYAN}AGENT_IP${RESET}       — IP de la Raspberry Pi en la red (10.42.0.1 si usas hotspot)"
    echo -e "    ${CYAN}AGENT_PORT${RESET}     — Puerto UDP del Agent (default: 8888)"
    echo ""

    local current_ip
    current_ip=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | head -1 || true)
    if [[ -n "$current_ip" ]]; then
        info "IP actual de esta Raspberry Pi: ${BOLD}$current_ip${RESET}"
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
    success "Credenciales WiFi guardadas."
    warn "Recuerda recompilar el firmware del ESP32 (opción 2) para aplicar los cambios."
    read -rp "  Presiona Enter para continuar..." _
}

# ── 1c. Credenciales Telegram ─────────────────────────────────────────────────
edit_telegram_env() {
    echo ""
    echo -e "${BOLD}  Credenciales — Notificaciones Telegram${RESET}"
    echo -e "  Archivo: ${YELLOW}database/.env${RESET}"
    echo ""
    echo -e "  Variables requeridas:"
    echo -e "    ${CYAN}TELEGRAM_BOT_TOKEN${RESET}  — Token del bot (obtenlo de @BotFather)"
    echo -e "    ${CYAN}TELEGRAM_CHAT_ID${RESET}    — Tu Chat ID (obtenlo de @userinfobot)"
    echo ""

    if [[ ! -f "$DB_ENV" ]]; then
        warn ".env de base de datos no encontrado."
        if [[ -f "$DB_ENV_EXAMPLE" ]]; then
            info "Creando database/.env desde plantilla..."
            cp "$DB_ENV_EXAMPLE" "$DB_ENV"
            success "Creado: database/.env"
        else
            error ".env.example no encontrado."
            read -rp "  Presiona Enter para volver..." _; return
        fi
    fi

    local cur_token cur_chat
    cur_token=$(grep "^TELEGRAM_BOT_TOKEN=" "$DB_ENV" | cut -d'=' -f2 | tr -d '[:space:]' || true)
    cur_chat=$(grep  "^TELEGRAM_CHAT_ID="   "$DB_ENV" | cut -d'=' -f2 | tr -d '[:space:]' || true)

    if [[ -n "$cur_token" ]]; then
        local masked_token="${cur_token:0:6}****${cur_token: -4}"
        info "Token actual : ${BOLD}$masked_token${RESET}"
    else
        warn "TELEGRAM_BOT_TOKEN no configurado."
    fi
    if [[ -n "$cur_chat" ]]; then
        info "Chat ID actual: ${BOLD}$cur_chat${RESET}"
    else
        warn "TELEGRAM_CHAT_ID no configurado."
    fi
    echo ""

    read -rsp "  Nuevo TELEGRAM_BOT_TOKEN (Enter para conservar): " new_token; echo ""
    read -rp  "  Nuevo TELEGRAM_CHAT_ID   (Enter para conservar): " new_chat
    echo ""

    _upsert_env() {
        local key="$1" val="$2" file="$3"
        if grep -q "^${key}=" "$file"; then
            sed -i "s|^${key}=.*|${key}=${val}|" "$file"
        else
            echo "${key}=${val}" >> "$file"
        fi
    }

    local changed=false
    if [[ -n "$new_token" ]]; then _upsert_env "TELEGRAM_BOT_TOKEN" "$new_token" "$DB_ENV"; changed=true; fi
    if [[ -n "$new_chat"  ]]; then _upsert_env "TELEGRAM_CHAT_ID"   "$new_chat"  "$DB_ENV"; changed=true; fi

    if $changed; then
        success "Credenciales de Telegram guardadas en database/.env"
        warn "Reinicia el servicio para aplicar: sudo systemctl restart smart-alerter"
    else
        info "Sin cambios."
    fi
    read -rp "  Presiona Enter para continuar..." _
}

# ── 1d. Configurar Hotspot WiFi ───────────────────────────────────────────────
configure_hotspot() {
    echo ""
    echo -e "${BOLD}  Configurar Hotspot WiFi — Red de Sensores ESP32${RESET}"
    echo ""
    echo -e "  Crea un punto de acceso WiFi en ${CYAN}wlan0${RESET} con IP ${CYAN}10.42.0.1${RESET}."
    echo -e "  Configura los ESP32 con ${CYAN}AGENT_IP=10.42.0.1${RESET} en el .env de WiFi (opción 1b)."
    echo ""

    # Verificar nmcli (NetworkManager) — instalar si no está disponible
    if ! command -v nmcli &>/dev/null; then
        warn "nmcli no encontrado. Instalando network-manager..."
        sudo apt-get update -qq
        sudo apt-get install -y network-manager
        sudo systemctl enable --now NetworkManager
        sleep 2  # Dar tiempo al servicio para arrancar
        if ! command -v nmcli &>/dev/null; then
            error "No se pudo instalar nmcli. Verifica tu conexión y permisos."
            read -rp "  Presiona Enter para volver..." _; return
        fi
        success "NetworkManager instalado y activo."
    fi

    # Leer SSID y contraseña del .env WiFi (si están configurados)
    local ssid="BioFloc-Sensors" pass="biofloc2024"
    if [[ -f "$WIFI_ENV" ]]; then
        local _ssid _pass
        _ssid=$(grep "^WIFI_SSID="     "$WIFI_ENV" | cut -d'=' -f2 | tr -d '[:space:]' || true)
        _pass=$(grep "^WIFI_PASSWORD=" "$WIFI_ENV" | cut -d'=' -f2 || true)
        [[ -n "$_ssid" && "$_ssid" != "TU_RED_WIFI"    ]] && ssid="$_ssid"
        [[ -n "$_pass" && "$_pass" != "TU_CONTRASEÑA"  ]] && pass="$_pass"
    fi
    # Nombre interno de la conexión NetworkManager (dinámico)
    local ap_con_name="${ssid}-AP"

    # Interfaz WiFi (configurable; por defecto wlan0)
    local interface="wlan0"

    echo -e "  ${BOLD}Configuración para el AP:${RESET}"
    echo -e "    SSID     : ${CYAN}$ssid${RESET}"
    echo -e "    Password : ${CYAN}$pass${RESET}"
    echo -e "    Interfaz : ${CYAN}$interface${RESET}"
    echo -e "    IP GW    : ${CYAN}10.42.0.1/24${RESET}"
    echo ""

    read -rp "  ¿Crear / actualizar el hotspot con esta configuración? [S/n]: " ans
    [[ "$ans" =~ ^[nN]$ ]] && { info "Operación cancelada."; read -rp "  Presiona Enter..." _; return; }

    # ── Pre-vuelo 1: Validación de hardware ──────────────────────────────────
    info "Verificando interfaz WiFi '$interface'..."
    if ! ip link show "$interface" &>/dev/null; then
        error "Interfaz '$interface' no encontrada."
        error "¿El hardware WiFi está conectado? Verifica con: ip link show"
        warn  "Si tu adaptador usa otro nombre (ej: wlp2s0), ajusta la variable 'interface' en este script."
        read -rp "  Presiona Enter para volver..." _; return
    fi
    success "  Interfaz '$interface' detectada."

    # ── Pre-vuelo 2: Delegar interfaz a NetworkManager (anti-Netplan) ─────────
    local netplan_nm="/etc/netplan/99-networkmanager.yaml"
    if [[ ! -f "$netplan_nm" ]]; then
        info "Delegando '$interface' a NetworkManager en Netplan..."
        sudo tee "$netplan_nm" > /dev/null <<'NETPLAN'
network:
  version: 2
  renderer: NetworkManager
NETPLAN
        sudo chmod 600 "$netplan_nm"
        info "Aplicando configuración Netplan..."
        sudo netplan apply 2>/dev/null || true
        sudo systemctl restart NetworkManager
        sleep 3  # Esperar que NM retome el control de las interfaces
        success "  Netplan configurado. NetworkManager controla '$interface'."
    else
        info "  Netplan ya delega a NetworkManager ($netplan_nm)."
    fi

    # ── Pre-vuelo 3: Liberar antena WiFi de conexiones previas ────────────────
    info "Liberando interfaz '$interface' de conexiones activas..."
    sudo nmcli device disconnect "$interface" 2>/dev/null || true
    sleep 1

    info "Eliminando conexión '${ap_con_name}' anterior (si existe)..."
    sudo nmcli con delete "${ap_con_name}" 2>/dev/null || true
    sleep 1

    info "Creando punto de acceso '${ap_con_name}'..."
    if sudo nmcli con add type wifi ifname "$interface" con-name "${ap_con_name}" autoconnect yes ssid "$ssid" \
        && sudo nmcli con modify "${ap_con_name}" \
            802-11-wireless.mode ap \
            802-11-wireless-security.key-mgmt wpa-psk \
            802-11-wireless-security.psk "$pass" \
            ipv4.method shared \
            ipv4.addresses "10.42.0.1/24" \
        && sudo nmcli con up "${ap_con_name}"; then
        echo ""
        success "════════════════════════════════════════════"
        success "  ¡Hotspot activo!  📡"
        success "  SSID: $ssid  |  GW: 10.42.0.1"
        success "════════════════════════════════════════════"
        info "Asegúrate de que AGENT_IP=10.42.0.1 en el .env de WiFi (opción 1b)."
    else
        error "No se pudo crear el punto de acceso."
        warn  "Verifica que '$interface' exista: ip link show $interface"
        warn  "Verifica que NetworkManager esté activo: systemctl status NetworkManager"
        warn  "Si Netplan acaba de aplicarse, intenta de nuevo en 10 segundos."
    fi
    echo ""
    read -rp "  Presiona Enter para continuar..." _
}

# ==============================================================================
# OPCIÓN 2 — Despliegue de Sensores (ESP32)
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
    info "Activando entorno ESP-IDF (sin ROS para evitar conflictos)..."
    source_idf
    "$script"
}

# ==============================================================================
# OPCIÓN 3 — Activación de Servicios
# ==============================================================================

# ── 3a. Deploy servicios systemd ─────────────────────────────────────────────
deploy_services() {
    echo ""
    echo -e "${BOLD}  Instalar servicios systemd — One-Click Deploy${RESET}"
    echo ""

    # Verificación 1: escritura en el FS
    info "Verificando escritura en el sistema de archivos..."
    local test_file
    test_file=$(mktemp 2>/dev/null) || {
        error "El sistema de archivos está en modo solo-lectura (¿SD dañada?)."
        error "Intenta: sudo mount -o remount,rw /"
        read -rp "  Presiona Enter para volver..." _
        return 1
    }
    rm -f "$test_file"
    success "Sistema de archivos disponible para escritura."

    # Verificación 2: dependencias
    info "Verificando dependencias..."
    local missing=0

    if ! dpkg -l iptables-persistent &>/dev/null 2>&1; then
        warn "  [!] iptables-persistent NO instalado."
        warn "      sudo apt-get install -y iptables-persistent"
        missing=$((missing + 1))
    else
        success "  iptables-persistent: OK"
    fi

    if ! python3 -c "import dotenv" &>/dev/null 2>&1; then
        warn "  [!] python3-dotenv NO disponible."
        warn "      pip3 install python-dotenv --break-system-packages"
        missing=$((missing + 1))
    else
        success "  python3-dotenv: OK"
    fi

    if ! python3 -c "import pymongo" &>/dev/null 2>&1; then
        warn "  [!] pymongo NO disponible."
        warn "      pip3 install pymongo --break-system-packages"
        missing=$((missing + 1))
    else
        success "  pymongo: OK"
    fi

    if [[ $missing -gt 0 ]]; then
        echo ""
        warn "$missing dependencia(s) faltante(s). ¿Continuar de todas formas? [s/N]"
        read -rp "  Respuesta: " ans
        [[ "$ans" =~ ^[sS]$ ]] || { info "Deploy cancelado."; read -rp "  Presiona Enter..." _; return; }
    fi

    # Dar permisos de ejecución
    info "Aplicando permisos de ejecución..."
    chmod +x "$SCRIPTS_DIR"/*.sh
    chmod +x "$SCRIPTS_DIR"/telegram/*.py 2>/dev/null || true
    success "Permisos aplicados."

    # Generar e instalar los .service desde plantillas
    local current_user="$USER"
    info "Usuario: ${BOLD}$current_user${RESET}  |  Scripts: ${BOLD}$SCRIPTS_DIR${RESET}"
    echo ""

    for tpl in "$SERVICES_DIR"/smart-*.service.tpl; do
        [[ -f "$tpl" ]] || continue
        local svc_name
        svc_name=$(basename "$tpl" .tpl)
        local dest="/etc/systemd/system/$svc_name"

        info "Generando $svc_name ..."
        sed \
            -e "s|{{USER}}|${current_user}|g" \
            -e "s|{{SCRIPTS_DIR}}|${SCRIPTS_DIR}|g" \
            -e "s|{{REPO_DIR}}|${REPO_DIR}|g" \
            "$tpl" | sudo tee "$dest" > /dev/null
        sudo chmod 644 "$dest"
        success "  → $dest"
    done

    # Recargar systemd y habilitar servicios
    echo ""
    info "Recargando daemon systemd..."
    sudo systemctl daemon-reload

    info "Habilitando e iniciando servicios..."
    sudo systemctl enable --now smart-agent.service smart-bridge.service smart-alerter.service

    # Alias de acceso rápido
    local alias_line="alias status='bash \"${SCRIPTS_DIR}/status.sh\"'"
    if grep -qF "alias status=" "$HOME/.bashrc" 2>/dev/null; then
        info "Alias 'status' ya existe en ~/.bashrc — sin cambios."
    else
        echo "" >> "$HOME/.bashrc"
        echo "# Gateway BioFloc — acceso rápido al panel de estado" >> "$HOME/.bashrc"
        echo "$alias_line" >> "$HOME/.bashrc"
        success "Alias 'status' añadido a ~/.bashrc"
    fi

    # Post-Deploy Health Check
    echo ""
    info "Esperando 5 segundos para que los servicios arranquen..."
    sleep 5

    local all_ok=true
    echo ""
    echo -e "${BOLD}  [ Post-Deploy Health Check ]${RESET}"

    for svc in smart-agent smart-bridge smart-alerter; do
        if systemctl is-active --quiet "$svc" 2>/dev/null; then
            success "  $svc: ${GREEN}ACTIVO ✓${RESET}"
        else
            error   "  $svc: ${RED}CAÍDO ✗${RESET}"
            echo ""
            warn "  Últimas 5 líneas de log para $svc:"
            echo -e "${BLUE}  ──────────────────────────────────────${RESET}"
            sudo journalctl -u "$svc" -n 5 --no-pager 2>/dev/null | sed 's/^/    /' || true
            echo -e "${BLUE}  ──────────────────────────────────────${RESET}"
            all_ok=false
        fi
    done

    echo ""
    if $all_ok; then
        success "════════════════════════════════════════════"
        success "  ¡Todos los servicios activos!  🟢"
        success "════════════════════════════════════════════"
        echo ""
        warn "¿Ver el panel de estado ahora? [S/n]"
        read -rp "  Respuesta: " view_ans
        if [[ ! "$view_ans" =~ ^[nN]$ ]]; then
            bash "$SCRIPTS_DIR/status.sh"; echo ""
        fi
    else
        warn "════════════════════════════════════════════"
        warn "  Uno o más servicios no arrancaron."
        warn "  Revisa con la opción  4 → b (Logs)."
        warn "════════════════════════════════════════════"
    fi
    read -rp "  Presiona Enter para continuar..." _
}

# ── 3b. Configurar Cron ───────────────────────────────────────────────────────
configure_cron() {
    echo ""
    echo -e "${BOLD}  Configurar Reportes Automáticos — Cron${RESET}"
    echo ""
    echo -e "  El script ${CYAN}smart_reporter.py${RESET} se ejecutará automáticamente"
    echo -e "  a las horas que configures. Cubre ventanas de 12 horas."
    echo ""

    local reporter_script="$SCRIPTS_DIR/telegram/smart_reporter.py"
    if [[ ! -f "$reporter_script" ]]; then
        error "No se encontró: $reporter_script"
        read -rp "  Presiona Enter para volver..." _; return
    fi

    local hora_manana hora_tarde
    while true; do
        read -rp "  Hora del reporte de MAÑANA [0-23, default: 8]: " hora_manana
        hora_manana="${hora_manana:-8}"
        if [[ "$hora_manana" =~ ^[0-9]+$ ]] && (( hora_manana >= 0 && hora_manana <= 23 )); then break; fi
        warn "Hora inválida. Ingresa un número entre 0 y 23."
    done

    while true; do
        read -rp "  Hora del reporte de TARDE  [0-23, default: 20]: " hora_tarde
        hora_tarde="${hora_tarde:-20}"
        if [[ "$hora_tarde" =~ ^[0-9]+$ ]] && (( hora_tarde >= 0 && hora_tarde <= 23 )); then break; fi
        warn "Hora inválida. Ingresa un número entre 0 y 23."
    done

    echo ""
    info "Configurando crontab:"
    info "  Reporte mañana : ${BOLD}${hora_manana}:00${RESET}"
    info "  Reporte tarde  : ${BOLD}${hora_tarde}:00${RESET}"
    echo ""

    local tmp_cron
    tmp_cron=$(mktemp)
    crontab -l 2>/dev/null | grep -v "smart_reporter\.py" > "$tmp_cron" || true
    echo "0 ${hora_manana} * * * /usr/bin/python3 ${reporter_script} >> /tmp/reporter.log 2>&1" >> "$tmp_cron"
    echo "0 ${hora_tarde}  * * * /usr/bin/python3 ${reporter_script} >> /tmp/reporter.log 2>&1" >> "$tmp_cron"
    crontab "$tmp_cron"
    rm -f "$tmp_cron"

    echo ""
    success "Crontab actualizado (sin duplicados)."
    echo ""
    echo -e "  ${BOLD}Entradas del reporter en crontab:${RESET}"
    crontab -l 2>/dev/null | grep "smart_reporter" | sed 's/^/    /'
    echo ""
    warn "Nota: horas en zona horaria del sistema."
    warn "Ajustar con: sudo dpkg-reconfigure tzdata"
    read -rp "  Presiona Enter para continuar..." _
}

# ── 3c. Aplicar Firewall ──────────────────────────────────────────────────────
apply_firewall() {
    echo ""
    echo -e "${BOLD}  Firewall — Modo Gateway Aislado${RESET}"
    echo ""
    echo -e "  ${YELLOW}Reglas iptables a aplicar:${RESET}"
    echo -e "    ${CYAN}IF_WAN${RESET} → Internet / Red Lab  — SSH permitido"
    echo -e "    ${CYAN}IF_LAN${RESET} → Red Sensores ESP32 — Solo DHCP, DNS, UDP"
    echo -e "    ${CYAN}FORWARD${RESET}              → DESACTIVADO (aislamiento total)"
    echo ""

    local fw_script="$SCRIPTS_DIR/firewall.sh"
    if [[ ! -f "$fw_script" ]]; then
        error "scripts/firewall.sh no encontrado."
        read -rp "  Presiona Enter para volver..." _; return
    fi

    warn "El script detectará las interfaces automáticamente y pedirá confirmación."
    read -rp "  ¿Ejecutar firewall.sh ahora? [s/N]: " ans
    if [[ "$ans" =~ ^[sS]$ ]]; then
        sudo bash "$fw_script"
    else
        info "Firewall no aplicado."
    fi
    echo ""
    read -rp "  Presiona Enter para continuar..." _
}

# ==============================================================================
# OPCIÓN 4 — Diagnóstico y Operación
# ==============================================================================

# ── 4a. Panel de Estado ───────────────────────────────────────────────────────
gateway_status() {
    echo ""
    local status_script="$SCRIPTS_DIR/status.sh"
    if [[ ! -f "$status_script" ]]; then
        error "scripts/status.sh no encontrado."
        read -rp "  Presiona Enter para volver..." _; return
    fi
    bash "$status_script"
    echo ""
    read -rp "  Presiona Enter para volver..." _
}

# ── 4b. Logs en tiempo real ───────────────────────────────────────────────────
follow_logs() {
    echo ""
    echo -e "${BOLD}  Logs en tiempo real — Gateway${RESET}"
    echo ""
    echo -e "  ${CYAN}a)${RESET}  smart-agent    (micro-ROS UDP)"
    echo -e "  ${CYAN}b)${RESET}  smart-bridge   (ROS → MongoDB)"
    echo -e "  ${CYAN}c)${RESET}  smart-alerter  (Alertas Telegram)"
    echo -e "  ${CYAN}d)${RESET}  Todos los servicios"
    echo ""
    read -rp "  Opción: " lopt
    echo ""
    info "Detener: Ctrl + C"
    echo ""
    case "$lopt" in
        a|A) sudo journalctl -u smart-agent   -f --no-pager || true ;;
        b|B) sudo journalctl -u smart-bridge  -f --no-pager || true ;;
        c|C) sudo journalctl -u smart-alerter -f --no-pager || true ;;
        d|D) sudo journalctl -u smart-agent -u smart-bridge -u smart-alerter -f --no-pager || true ;;
        *)   warn "Opción no válida." ;;
    esac
    echo ""
    read -rp "  Presiona Enter para continuar..." _
}

# ── 4c. Monitor de Tópicos ROS ────────────────────────────────────────────────
monitor_topics() {
    echo ""
    echo -e "${BOLD}  Monitor de Tópicos ROS 2${RESET}"
    echo ""
    info "Cargando entorno ROS 2..."

    if ! source_ros; then
        warn "Asegúrate de tener ROS 2 instalado (opción 0) y el Agent corriendo (opción 3a)."
        read -rp "  Presiona Enter para volver..." _; return
    fi

    echo ""
    echo -e "  ${BOLD}Tópicos activos en este momento:${RESET}"
    echo ""
    if ! ros2 topic list 2>/dev/null; then
        warn "No se encontraron tópicos. ¿Está corriendo smart-agent y el ESP32?"
        read -rp "  Presiona Enter para volver..." _; return
    fi

    echo ""
    echo -e "  ${BOLD}Opciones:${RESET}"
    echo -e "    ${CYAN}e)${RESET}  Escuchar datos en vivo de un tópico (ros2 topic echo)"
    echo -e "    ${CYAN}h)${RESET}  Ver frecuencia de publicación      (ros2 topic hz)"
    echo -e "    ${CYAN}0)${RESET}  Volver"
    echo ""
    read -rp "  Opción: " topt
    case "$topt" in
        e|E)
            read -rp "  Nombre del tópico (ej: /temperatura): " topic_name
            if [[ -n "$topic_name" ]]; then
                echo ""
                info "ros2 topic echo $topic_name  |  Detener: Ctrl + C"
                echo ""
                ros2 topic echo "$topic_name" || true
            fi
            ;;
        h|H)
            read -rp "  Nombre del tópico (ej: /temperatura): " topic_name
            if [[ -n "$topic_name" ]]; then
                echo ""
                info "ros2 topic hz $topic_name  |  Detener: Ctrl + C (10 seg)"
                echo ""
                ros2 topic hz "$topic_name" || true
            fi
            ;;
        0) return ;;
        *) warn "Opción no válida." ;;
    esac
    echo ""
    read -rp "  Presiona Enter para continuar..." _
}

# ==============================================================================
# Submenús
# ==============================================================================

# ── Submenú: Configuración (1) ────────────────────────────────────────────────
menu_config() {
    while true; do
        show_header
        echo -e "  ${BOLD}1. Configuración del Sistema${RESET}"
        echo ""
        echo "    a)  Base de datos MongoDB   (database/.env)"
        echo "    b)  WiFi / micro-ROS        (microros-esp/main/versions/wifi/.env)"
        echo "    c)  Telegram                (Bot Token / Chat ID)"
        echo "    d)  Hotspot WiFi            (Crear red de sensores en wlan0)"
        echo ""
        echo "    0)  Volver"
        echo ""
        read -rp "  Opción: " opt
        case "$opt" in
            a|A) edit_db_env ;;
            b|B) edit_wifi_env ;;
            c|C) edit_telegram_env ;;
            d|D) configure_hotspot ;;
            0)   return ;;
            *)   warn "Opción no válida." ; sleep 1 ;;
        esac
    done
}

# ── Submenú: Activación de Servicios (3) ─────────────────────────────────────
menu_services() {
    while true; do
        show_header
        echo -e "  ${BOLD}3. Activación de Servicios${RESET}"
        echo ""
        echo "    a)  Desplegar Servicios Systemd   (agent + bridge + alerter)"
        echo "    b)  Configurar Reportes Telegram  (Cron — horarios automáticos)"
        echo "    c)  Aplicar Reglas de Firewall    (auto-detección de interfaces)"
        echo ""
        echo "    0)  Volver"
        echo ""
        read -rp "  Opción: " opt
        case "$opt" in
            a|A) deploy_services ;;
            b|B) configure_cron ;;
            c|C) apply_firewall ;;
            0)   return ;;
            *)   warn "Opción no válida." ; sleep 1 ;;
        esac
    done
}

# ── Submenú: Diagnóstico y Operación (4) ─────────────────────────────────────
menu_diagnostics() {
    while true; do
        show_header
        echo -e "  ${BOLD}4. Diagnóstico y Operación${RESET}"
        echo ""
        echo "    a)  Panel de Estado          (servicios, red, hardware, logs rápidos)"
        echo "    b)  Logs en tiempo real       (journalctl — agent / bridge / alerter)"
        echo "    c)  Monitor de Tópicos ROS    (datos crudos de los ESP32)"
        echo ""
        echo "    0)  Volver"
        echo ""
        read -rp "  Opción: " opt
        case "$opt" in
            a|A) gateway_status ;;
            b|B) follow_logs ;;
            c|C) monitor_topics ;;
            0)   return ;;
            *)   warn "Opción no válida." ; sleep 1 ;;
        esac
    done
}

# ==============================================================================
# Menú Principal
# ==============================================================================
main_menu() {
    while true; do
        show_header
        echo "  0)  Instalación de Dependencias    (ROS 2, ESP-IDF, micro-ROS)"
        echo "  1)  Configuración del Sistema       (MongoDB, WiFi, Telegram, Hotspot)"
        echo "  2)  Despliegue de Sensores          (Compilar y flashear ESP32)"
        echo "  3)  Activación de Servicios         (Systemd, Cron, Firewall)"
        echo "  4)  Diagnóstico y Operación         (Estado, Logs, Tópicos ROS)"
        echo ""
        echo "  q)  Salir"
        echo ""
        read -rp "  Opción: " opt
        case "$opt" in
            0) run_installer ;;
            1) menu_config ;;
            2) launch_esp32 ;;
            3) menu_services ;;
            4) menu_diagnostics ;;
            q|Q) echo "" ; info "¡Suerte con esas ESP32!" ; echo "" ; exit 0 ;;
            *) warn "Opción no válida." ; sleep 1 ;;
        esac
    done
}

main_menu
