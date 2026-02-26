#!/usr/bin/env bash
# ==============================================================================
# install.sh — Monitor de Microalgas UCN
# Instalación nativa en Ubuntu 24.04 (ROS 2 Jazzy + micro-ROS + Python deps)
#
# Uso:
#   chmod +x install.sh && ./install.sh
#
# Variables de entorno opcionales:
#   REPO_URL    URL del repositorio git (por defecto: GitHub oficial)
#   REPO_BRANCH Rama a clonar (por defecto: main)
#   INSTALL_DIR Directorio de instalación (por defecto: ~/Microalgas-Monitoring)
# ==============================================================================

set -euo pipefail

# ─── Colores ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${CYAN}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}[ OK ]${RESET}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*"; exit 1; }
header()  { echo -e "\n${BOLD}${CYAN}══════════════════════════════════════════${RESET}"
            echo -e "${BOLD}${CYAN}  $*${RESET}"
            echo -e "${BOLD}${CYAN}══════════════════════════════════════════${RESET}"; }

# ─── Usuario real (funciona con o sin sudo) ───────────────────────────────────
REAL_USER="${SUDO_USER:-$USER}"
REAL_HOME=$(getent passwd "$REAL_USER" | cut -d: -f6)

# Si el home fue borrado, recrearlo
if [[ ! -d "$REAL_HOME" ]]; then
    info "Directorio home de $REAL_USER no existe — creando $REAL_HOME ..."
    mkdir -p "$REAL_HOME"
    chown "$REAL_USER:$REAL_USER" "$REAL_HOME"
    chmod 755 "$REAL_HOME"
fi

# ─── Variables configurables ──────────────────────────────────────────────────
REPO_URL="${REPO_URL:-https://github.com/Menderin/Monitoreo-de-Sensores---Implementacion-ROS}"
REPO_BRANCH="${REPO_BRANCH:-main}"
INSTALL_DIR="${INSTALL_DIR:-$REAL_HOME/Microalgas-Monitoring}"
ROS_DISTRO="jazzy"
MICROROS_WS="$REAL_HOME/microros_ws"
ESP_IDF_VERSION="${ESP_IDF_VERSION:-v5.4.1}"
ESP_IDF_DIR="$REAL_HOME/esp/esp-idf"

# ==============================================================================
# 0. Permisos de acceso a puertos serie (ESP32)
# ==============================================================================
if ! groups "$REAL_USER" | grep -qw dialout; then
    info "Añadiendo $REAL_USER al grupo dialout (acceso a puertos serie)..."
    usermod -aG dialout "$REAL_USER"
    success "Usuario añadido al grupo dialout (requiere cerrar sesión para aplicar)."
else
    success "$REAL_USER ya pertenece al grupo dialout."
fi

# ==============================================================================
# 1. Verificar Ubuntu 24.04
# ==============================================================================
header "1/7  Verificando sistema operativo"

[[ -f /etc/os-release ]] || error "No se puede determinar el sistema operativo."
source /etc/os-release

[[ "$ID" == "ubuntu" ]] || error "Este script requiere Ubuntu 24.04. Sistema detectado: $ID"
[[ "$VERSION_ID" == "24.04" ]] || {
    warn "Ubuntu $VERSION_ID detectado. Este script está optimizado para 24.04."
    read -rp "¿Continuar de todas formas? [s/N] " ans
    [[ "$ans" =~ ^[sS]$ ]] || error "Instalación cancelada."
}

success "Ubuntu $VERSION_ID detectado."

# ==============================================================================
# 2. Instalar ROS 2 Jazzy
# ==============================================================================
header "2/7  Instalando ROS 2 Jazzy"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    success "ROS 2 Jazzy ya está instalado."
else
    info "Configurando repositorio de ROS 2..."

    sudo apt-get update -qq
    sudo apt-get install -y --no-install-recommends \
        software-properties-common curl gnupg lsb-release

    # Locale UTF-8 (requerido por ROS)
    sudo apt-get install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Repositorio oficial de ROS 2
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        https://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt-get update -qq

    info "Instalando ros-jazzy-ros-base (puede tardar varios minutos)..."
    sudo apt-get install -y \
        ros-jazzy-ros-base \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-std-msgs \
        ros-jazzy-ros2cli \
        ros-jazzy-ros2run \
        ros-jazzy-ros2topic \
        ros-jazzy-rcl \
        ros-jazzy-rclpy \
        python3-colcon-common-extensions \
        python3-rosdep

    # Inicializar rosdep
    rosdep init 2>/dev/null || true
    sudo -u "$REAL_USER" rosdep update

    # Añadir source automático al .bashrc del usuario real
    touch "$REAL_HOME/.bashrc"
    if ! grep -q "source /opt/ros/jazzy/setup.bash" "$REAL_HOME/.bashrc"; then
        echo "source /opt/ros/jazzy/setup.bash" >> "$REAL_HOME/.bashrc"
        info "Añadido 'source /opt/ros/jazzy/setup.bash' al ~/.bashrc"
    fi

    success "ROS 2 Jazzy instalado."
fi

# Source ROS para el resto del script
# shellcheck disable=SC1091
set +u; source /opt/ros/jazzy/setup.bash; set -u

# ==============================================================================
# 3. Instalar micro-ROS Agent (compilar desde fuente con colcon)
# ==============================================================================
header "3/7  Instalando micro-ROS Agent"

# Verificar si ya existe y funciona
if [[ -f "$MICROROS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "$MICROROS_WS/install/setup.bash"; set -u
    if command -v micro_ros_agent &>/dev/null || ros2 pkg list 2>/dev/null | grep -q "micro_ros_agent"; then
        success "micro-ROS Agent ya instalado en $MICROROS_WS"
    else
        warn "Workspace existe pero el Agent no está disponible — recompilando..."
        
        info "Limpiando build artifacts residuales..."
        rm -rf "$MICROROS_WS/build" "$MICROROS_WS/install" "$MICROROS_WS/log"
        
        info "Resolviendo dependencias con rosdep..."
        sudo -u "$REAL_USER" bash -c "source /opt/ros/jazzy/setup.bash && cd '$MICROROS_WS' && rosdep install --from-paths src --ignore-src -y" || true
        
        info "Compilando con modo secuencial (optimizado para Raspberry Pi)..."
        if sudo -u "$REAL_USER" bash -c "source /opt/ros/jazzy/setup.bash && cd '$MICROROS_WS' && colcon build --symlink-install --executor sequential --parallel-workers 1"; then
            set +u; source "$MICROROS_WS/install/setup.bash"; set -u
            success "micro-ROS Agent recompilado exitosamente."
        else
            error "Falló la recompilación del micro-ROS Agent."
            error "Revisa los logs en $MICROROS_WS/log/latest_build/"
            warn "Intenta: rm -rf $MICROROS_WS/build/* y ejecuta de nuevo ./install.sh"
            exit 1
        fi
    fi
else
    info "Instalando dependencias de compilación..."
    sudo apt-get update -qq
    sudo apt-get install -y --no-install-recommends \
        python3-pip \
        python3-dev \
        python3-setuptools \
        python3-colcon-common-extensions \
        ros-dev-tools \
        build-essential

    info "Creando workspace en $MICROROS_WS ..."
    mkdir -p "$MICROROS_WS/src"
    chown -R "$REAL_USER:$REAL_USER" "$MICROROS_WS"
    pushd "$MICROROS_WS/src" > /dev/null

    info "Clonando micro_ros_msgs y micro-ROS-Agent (rama jazzy)..."
    sudo -u "$REAL_USER" git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git
    sudo -u "$REAL_USER" git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

    cd "$MICROROS_WS"
    
    info "Resolviendo dependencias con rosdep..."
    sudo -u "$REAL_USER" bash -c "source /opt/ros/jazzy/setup.bash && cd '$MICROROS_WS' && rosdep install --from-paths src --ignore-src -y" || true
    
    info "Compilando (puede tardar 5-15 minutos en Raspberry Pi)..."
    info "Usando compilación secuencial para evitar problemas de memoria..."
    
    # Compilar con flags optimizados para ARM/Raspberry Pi
    if sudo -u "$REAL_USER" bash -c "source /opt/ros/jazzy/setup.bash && cd '$MICROROS_WS' && colcon build --symlink-install --executor sequential --parallel-workers 1"; then
        success "micro-ROS Agent compilado exitosamente."
    else
        error "Falló la compilación del micro-ROS Agent."
        error "Revisa los logs en $MICROROS_WS/log/latest_build/"
        error "Logs de stderr en: $MICROROS_WS/log/latest_build/micro_ros_msgs/stderr.log"
        popd > /dev/null
        exit 1
    fi

    popd > /dev/null

    # shellcheck disable=SC1091
    set +u; source "$MICROROS_WS/install/setup.bash"; set -u
    success "micro-ROS Agent instalado."

    # Añadir source automático al .bashrc del usuario real
    if ! grep -q "microros_ws/install/setup.bash" "$REAL_HOME/.bashrc"; then
        echo "source $MICROROS_WS/install/setup.bash" >> "$REAL_HOME/.bashrc"
        info "Añadido 'source $MICROROS_WS/install/setup.bash' al ~/.bashrc"
    fi
fi

# ==============================================================================
# 4. Instalar ESP-IDF
# ==============================================================================
header "4/7  Instalando ESP-IDF ($ESP_IDF_VERSION)"

# Detección de Raspberry Pi — ESP-IDF no es necesario en la RPi
_is_rpi=false
if grep -qi "raspberry\|bcm2\|rpi" /proc/cpuinfo 2>/dev/null || [[ "$(uname -m)" == "aarch64" && -f /sys/firmware/devicetree/base/model && "$(cat /sys/firmware/devicetree/base/model 2>/dev/null)" == *"Raspberry"* ]]; then
    _is_rpi=true
fi

if $_is_rpi; then
    warn "Raspberry Pi detectada — omitiendo ESP-IDF."
    warn "ESP-IDF solo es necesario en el PC de desarrollo para compilar/flashear el ESP32."
    info "Si necesitas ESP-IDF en la RPi, ejecuta: ESP_FORCE_IDF=1 sudo ./install.sh"
    if [[ "${ESP_FORCE_IDF:-0}" != "1" ]]; then
        success "ESP-IDF omitido (no necesario en Raspberry Pi)."
    fi
fi

if ! $_is_rpi || [[ "${ESP_FORCE_IDF:-0}" == "1" ]]; then
    _idf_py_env=$(ls -d "$REAL_HOME/.espressif/python_env"/idf*/bin/python3 2>/dev/null | head -1 || true)
    if [[ -f "$ESP_IDF_DIR/export.sh" && -n "$_idf_py_env" ]]; then
        success "ESP-IDF ya instalado en $ESP_IDF_DIR"
    else
    info "Instalando dependencias del sistema para ESP-IDF..."
    apt-get install -y --no-install-recommends \
        git wget flex bison gperf cmake ninja-build ccache \
        libffi-dev libssl-dev dfu-util libusb-1.0-0 \
        python3-venv python3-pip

    if [[ ! -f "$ESP_IDF_DIR/export.sh" ]]; then
        info "Clonando ESP-IDF $ESP_IDF_VERSION en $ESP_IDF_DIR ..."
        mkdir -p "$REAL_HOME/esp"
        chown "$REAL_USER:$REAL_USER" "$REAL_HOME/esp"
        sudo -u "$REAL_USER" git clone -b "$ESP_IDF_VERSION" --recursive \
            https://github.com/espressif/esp-idf.git "$ESP_IDF_DIR"
    else
        info "Repositorio ESP-IDF ya clonado — completando instalación de herramientas..."
    fi

    info "Ejecutando install.sh de ESP-IDF para esp32..."
    sudo -u "$REAL_USER" bash -c "cd '$ESP_IDF_DIR' && ./install.sh esp32"

    info "Instalando paquetes Python requeridos por micro_ros_espidf_component..."
    _idf_pip=$(ls "$REAL_HOME/.espressif/python_env"/idf*/bin/pip 2>/dev/null | head -1 || true)
    if [[ -n "$_idf_pip" ]]; then
        sudo -u "$REAL_USER" "$_idf_pip" install --quiet \
            catkin_pkg empy lark colcon-common-extensions
        success "Paquetes micro-ROS instalados en el venv de ESP-IDF."
    else
        warn "No se encontró pip en el venv de ESP-IDF. Instálalos manualmente:"
        warn "  ~/.espressif/python_env/idf*/bin/pip install catkin_pkg empy lark colcon-common-extensions"
    fi

    if ! grep -q "esp-idf/export.sh" "$REAL_HOME/.bashrc"; then
        echo ". $ESP_IDF_DIR/export.sh" >> "$REAL_HOME/.bashrc"
        info "Añadido '. $ESP_IDF_DIR/export.sh' al ~/.bashrc"
    fi
    success "ESP-IDF instalado."
fi
fi  # cierre de: if ! $_is_rpi || ESP_FORCE_IDF

# ==============================================================================
# 5. Instalar dependencias Python
# ==============================================================================
header "5/7  Instalando dependencias Python"

# Usar pip con --break-system-packages (Ubuntu 24.04 PEP 668)
PIP_FLAGS="--break-system-packages --quiet"

info "Instalando pymongo, python-dotenv, certifi, numpy, pyyaml..."
pip3 install $PIP_FLAGS \
    "pymongo>=4.0.0" \
    "python-dotenv>=1.0.0" \
    "certifi" \
    "numpy>=1.24.0" \
    "pyyaml>=6.0"

# Verificar importaciones críticas
python3 -c "import pymongo; import dotenv; import certifi; import numpy; import yaml" \
    && success "Dependencias Python verificadas." \
    || error "Falló la verificación de dependencias Python."

# ==============================================================================
# 6. Clonar / verificar repositorio
# ==============================================================================
header "6/7  Preparando repositorio"

# Detectar si ya estamos dentro del proyecto
if [[ -f "$(pwd)/database/ros_sensor_node.py" && -d "$(pwd)/.git" ]]; then
    INSTALL_DIR="$(pwd)"
    success "Ejecutando desde dentro del proyecto: $INSTALL_DIR"
    info "Actualizando con git pull..."
    git pull --ff-only || warn "No se pudo hacer pull. ¿Hay cambios locales?"
elif [[ -d "$INSTALL_DIR/.git" ]]; then
    success "Repositorio encontrado en $INSTALL_DIR"
    info "Actualizando con git pull..."
    git -C "$INSTALL_DIR" pull --ff-only || warn "No se pudo hacer pull. ¿Hay cambios locales?"
else
    info "Clonando repositorio (rama: $REPO_BRANCH) en $INSTALL_DIR ..."
    git clone -b "$REPO_BRANCH" "$REPO_URL" "$INSTALL_DIR"
    success "Repositorio clonado."
fi

cd "$INSTALL_DIR"

# ==============================================================================
# 7. Configurar credenciales MongoDB
# ==============================================================================
header "7/7  Configuración de credenciales"

if [[ -f "database/.env" ]]; then
    success "database/.env ya existe."
    if grep -qE "^MONGO_URI=mongodb(\+srv)?://.+" database/.env; then
        success "MONGO_URI configurado."
    else
        warn "MONGO_URI en database/.env parece vacío o incorrecto."
        warn "Edítalo manualmente: nano $INSTALL_DIR/database/.env"
    fi
else
    if [[ -f ".env.example" ]]; then
        sudo -u "$REAL_USER" cp .env.example database/.env
        chown "$REAL_USER:$REAL_USER" database/.env
        chmod 600 database/.env
        echo ""
        echo -e "${YELLOW}  ┌──────────────────────────────────────────────────────┐${RESET}"
        echo -e "${YELLOW}  │  ACCIÓN REQUERIDA: Configurar database/.env           │${RESET}"
        echo -e "${YELLOW}  │                                                        │${RESET}"
        echo -e "${YELLOW}  │  Necesitas rellenar al menos:                          │${RESET}"
        echo -e "${YELLOW}  │    MONGO_URI=mongodb+srv://usuario:pass@cluster...     │${RESET}"
        echo -e "${YELLOW}  │    MONGO_DB=Datos_ESP                                  │${RESET}"
        echo -e "${YELLOW}  └──────────────────────────────────────────────────────┘${RESET}"
        echo ""
        read -rp "¿Editar database/.env ahora? [S/n] " edit_env
        if [[ ! "$edit_env" =~ ^[nN]$ ]]; then
            sudo -u "$REAL_USER" "${EDITOR:-nano}" database/.env
        fi
    else
        error "No se encontró .env.example. El repositorio puede estar incompleto."
    fi
fi

# ==============================================================================
# 8. Configurar hotspot WiFi para compatibilidad con ESP32
# ==============================================================================
header "8/8  Configurando hotspot WiFi (compatibilidad ESP32)"

# Solo aplica si hay NetworkManager y algún hotspot activo en modo AP
if command -v nmcli &>/dev/null; then
    # Buscar conexiones en modo Access Point (hotspot)
    HOTSPOT_NAMES=$(nmcli -t -f NAME,TYPE,802-11-wireless.mode con show 2>/dev/null \
        | awk -F: '$2=="wifi" && $3=="ap" {print $1}' || true)

    # Si no encuentra con el campo mode, buscar por las que tienen ssid y son wifi
    if [[ -z "$HOTSPOT_NAMES" ]]; then
        HOTSPOT_NAMES=$(nmcli -t -f NAME,TYPE con show 2>/dev/null \
            | awk -F: '$2=="802-11-wireless" {print $1}' || true)
        # Filtrar solo las que están en modo AP
        FILTERED=""
        for CON in $HOTSPOT_NAMES; do
            MODE=$(nmcli -f 802-11-wireless.mode con show "$CON" 2>/dev/null \
                | awk '{print $2}' || true)
            if [[ "$MODE" == "ap" ]]; then
                FILTERED="$FILTERED $CON"
            fi
        done
        HOTSPOT_NAMES="$FILTERED"
    fi

    if [[ -n "$HOTSPOT_NAMES" ]]; then
        for CON in $HOTSPOT_NAMES; do
            CON=$(echo "$CON" | xargs)  # trim espacios
            [[ -z "$CON" ]] && continue
            info "Configurando hotspot '${CON}' para compatibilidad ESP32..."

            # Aplicar: WPA2 puro + PMF desactivado + cifrado CCMP
            nmcli con modify "$CON" \
                802-11-wireless-security.key-mgmt   wpa-psk \
                802-11-wireless-security.proto      rsn \
                802-11-wireless-security.pairwise   ccmp \
                802-11-wireless-security.group      ccmp \
                802-11-wireless-security.pmf        0 \
                802-11-wireless.band                bg \
                2>/dev/null && success "'${CON}': WPA2-PSK puro, PMF desactivado, banda 2.4GHz" \
                            || warn  "No se pudo modificar '${CON}' (¿permisos de root?)."

            # Reiniciar el hotspot si está activo para aplicar cambios
            IS_ACTIVE=$(nmcli -t -f NAME con show --active 2>/dev/null \
                | grep -Fx "$CON" || true)
            if [[ -n "$IS_ACTIVE" ]]; then
                info "Reiniciando hotspot '${CON}' para aplicar cambios..."
                nmcli con down "$CON" 2>/dev/null || true
                sleep 2
                nmcli con up   "$CON" 2>/dev/null \
                    && success "Hotspot '${CON}' reiniciado." \
                    || warn    "No se pudo reiniciar '${CON}'. Reinicia manualmente."
            fi
        done
    else
        info "No se detectaron hotspots WiFi activos — omitiendo configuración ESP32."
        info "Si configuras un hotspot después, ejecuta:"
        info "  sudo nmcli con modify <NOMBRE_HOTSPOT> \\"
        info "    802-11-wireless-security.pmf 0 \\"
        info "    802-11-wireless-security.proto rsn \\"
        info "    802-11-wireless-security.pairwise ccmp \\"
        info "    802-11-wireless-security.group ccmp"
    fi
else
    info "NetworkManager no disponible — omitiendo configuración de hotspot."
fi

# ==============================================================================
# Resumen final
# ==============================================================================
echo ""
success "══════════════════════════════════════════════"
success "  ¡Instalación completada!"
success "══════════════════════════════════════════════"
echo ""
echo "  Proyecto instalado en: ${BOLD}$INSTALL_DIR${RESET}"
echo ""
echo -e "  ${BOLD}Para iniciar el sistema (en cada terminal nueva):${RESET}"
echo ""
echo -e "  ${CYAN}# 1. Cargar entornos ROS 2 + micro-ROS${RESET}"
echo -e "  source /opt/ros/jazzy/setup.bash"
echo -e "  source $MICROROS_WS/install/setup.bash"
echo ""
echo -e "  ${CYAN}# 2. Iniciar micro-ROS Agent (espera UDP del ESP32 en puerto 8888)${RESET}"
echo -e "  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
echo ""
echo -e "  ${CYAN}# 3. En otra terminal: iniciar nodo ROS 2 → MongoDB${RESET}"
echo -e "  source /opt/ros/jazzy/setup.bash"
echo -e "  source $MICROROS_WS/install/setup.bash"
echo -e "  python3 $INSTALL_DIR/database/ros_sensor_node.py"
echo ""
if $_is_rpi; then
    echo -e "  ${YELLOW}Nota: ESP-IDF no se instaló (no es necesario en Raspberry Pi).${RESET}"
    echo -e "  ${YELLOW}Compila y flashea el ESP32 desde tu PC de desarrollo.${RESET}"
else
    echo -e "  ${CYAN}# Para compilar/flashear el ESP32:${RESET}"
    echo -e "  . $ESP_IDF_DIR/export.sh"
    echo -e "  cd $INSTALL_DIR && ./menu.sh  # opción 3"
fi
echo ""
echo -e "  ${CYAN}# IP de este equipo (configurar como AGENT_IP en el ESP32):${RESET}"
ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v 127.0.0.1 | \
    while read -r ip; do echo -e "    ${GREEN}$ip${RESET}"; done
echo ""
echo -e "  ${YELLOW}Nota: Los sources ya fueron añadidos al ~/.bashrc.${RESET}"
echo -e "  ${YELLOW}Abre una terminal nueva o ejecuta: source ~/.bashrc${RESET}"
echo ""
