#!/usr/bin/env bash
# ==============================================================================
# install.sh — IoT Gateway Monitoring System
# Instalación nativa en Ubuntu 24.04 (ROS 2 Jazzy + micro-ROS + Python deps)
#
# Uso:
#   chmod +x install.sh && ./install.sh
#
# Variables de entorno opcionales:
#   REPO_URL    URL del repositorio git (por defecto: GitHub oficial)
#   REPO_BRANCH Rama a clonar (por defecto: main)
#   INSTALL_DIR Directorio de instalación (por defecto: ~/IoT-Gateway)
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

# ─── Opciones APT para redes restrictivas (CGNAT / universitarias) ────────────
# ForceIPv4   : evita bloqueos IPv6 en redes móviles/universitarias.
# Para redes con problemas de certificados: INSECURE_APT=1 ./install.sh
APT_OPTS="-o Acquire::ForceIPv4=true"
if [[ "${INSECURE_APT:-0}" == "1" ]]; then
    warn "INSECURE_APT=1 → Verificación TLS de APT deshabilitada (no recomendado)"
    APT_OPTS+=" -o Acquire::https::Verify-Peer=false -o Acquire::https::Verify-Host=false"
fi

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
INSTALL_DIR="${INSTALL_DIR:-$REAL_HOME/IoT-Gateway}"
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
header "1/8  Verificando sistema operativo"

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
header "2/8  Instalando ROS 2 Jazzy"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    success "ROS 2 Jazzy ya está instalado."
else
    info "Configurando repositorio de ROS 2..."

    if ! sudo apt-get $APT_OPTS update -qq 2>&1; then
        echo ""
        error_msg="apt update falló. Posibles causas:"
        error_msg+="\n  1. Sin conexión a Internet."
        error_msg+="\n  2. Un proxy/router está interceptando el tráfico HTTPS (MITM)."
        error_msg+="\n  3. Los certificados del sistema están desactualizados."
        error_msg+="\n"
        error_msg+="\n  Si estás en una red universitaria/corporativa con inspección de tráfico,"
        error_msg+="\n  puedes forzar la instalación bajo tu responsabilidad con:"
        error_msg+="\n"
        error_msg+="\n    INSECURE_APT=1 sudo -E ./install.sh"
        error_msg+="\n"
        error_msg+="\n  ⚠️  Esto desactiva la verificación TLS de APT (NO recomendado)."
        error "$error_msg"
    fi
    sudo apt-get $APT_OPTS install -y --no-install-recommends \
        software-properties-common curl gnupg lsb-release

    # Locale UTF-8 (requerido por ROS)
    sudo apt-get $APT_OPTS install -y locales
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

    sudo apt-get $APT_OPTS update -qq

    info "Instalando ros-jazzy-ros-base (puede tardar varios minutos)..."
    sudo apt-get $APT_OPTS --fix-broken install -y
    sudo apt-get $APT_OPTS install -y \
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
header "3/8  Instalando micro-ROS Agent"

# Verificar si ya existe y funciona
if [[ -f "$MICROROS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "$MICROROS_WS/install/setup.bash"; set -u
    if command -v micro_ros_agent &>/dev/null || ros2 pkg list 2>/dev/null | grep -q "micro_ros_agent"; then
        success "micro-ROS Agent ya instalado en $MICROROS_WS"
    else
        warn "Workspace existe pero el Agent no está disponible — recompilando..."
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
    sudo apt-get $APT_OPTS update -qq
    sudo apt-get $APT_OPTS install -y --no-install-recommends \
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
header "4/8  Instalando ESP-IDF ($ESP_IDF_VERSION)"

_idf_py_env=$(ls -d "$REAL_HOME/.espressif/python_env"/idf*/bin/python3 2>/dev/null | head -1 || true)
if [[ -f "$ESP_IDF_DIR/export.sh" && -n "$_idf_py_env" ]]; then
    success "ESP-IDF ya instalado en $ESP_IDF_DIR"
else
    info "Instalando dependencias del sistema para ESP-IDF..."
    sudo apt-get $APT_OPTS --fix-broken install -y
    sudo apt-get $APT_OPTS install -y --no-install-recommends \
        git wget flex bison gperf cmake ninja-build ccache \
        libffi-dev libssl-dev dfu-util libusb-1.0-0 \
        python3-venv python3-pip

    if [[ ! -f "$ESP_IDF_DIR/export.sh" ]]; then
        info "Clonando ESP-IDF $ESP_IDF_VERSION (shallow — sin historial)..."
        mkdir -p "$REAL_HOME/esp"
        chown "$REAL_USER:$REAL_USER" "$REAL_HOME/esp"
        sudo -u "$REAL_USER" git clone -b "$ESP_IDF_VERSION" --depth 1 --shallow-submodules --recursive \
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

# ==============================================================================
# 5. Instalar dependencias Python
# ==============================================================================
header "5/8  Instalando dependencias Python"

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
header "6/8  Preparando repositorio"

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
header "7/8  Configuración de credenciales"

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
# Resumen final
# ==============================================================================
# ==============================================================================
# 8/8  Configurar entorno, permisos y accesos directos
# ==============================================================================
header "8/8  Configurando entorno del sistema"

# ── 8a. Inyectar sources en ~/.bashrc (anti-duplicado) ─────────────────────────
_bashrc="$REAL_HOME/.bashrc"
_ros_source="source /opt/ros/jazzy/setup.bash"
_uros_source="source $MICROROS_WS/install/setup.bash"
_dds_export="export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"

info "Verificando entradas en $_bashrc ..."

if ! grep -qF "$_ros_source" "$_bashrc" 2>/dev/null; then
    echo ""                                                   >> "$_bashrc"
    echo "# ── ROS 2 Jazzy — añadido por install.sh ──────" >> "$_bashrc"
    echo "$_ros_source"                                       >> "$_bashrc"
    success "  Añadido: $_ros_source"
else
    info "  Ya presente: $_ros_source"
fi

if ! grep -qF "$_uros_source" "$_bashrc" 2>/dev/null; then
    echo "# ── micro-ROS workspace ───────────────────────" >> "$_bashrc"
    echo "$_uros_source"                                     >> "$_bashrc"
    success "  Añadido: $_uros_source"
else
    info "  Ya presente: $_uros_source"
fi

# Aislamiento DDS: fuerza ROS 2 a usar solo loopback (sin bucles multi-interfaz)
if ! grep -qF "ROS_AUTOMATIC_DISCOVERY_RANGE" "$_bashrc" 2>/dev/null; then
    echo "# ── Aislamiento DDS (solo loopback) ────────────" >> "$_bashrc"
    echo "$_dds_export"                                       >> "$_bashrc"
    success "  Añadido: $_dds_export"
else
    info "  Ya presente: ROS_AUTOMATIC_DISCOVERY_RANGE"
fi

# Asegurar que el .bashrc pertenece al usuario real (no a root)
chown "$REAL_USER:$REAL_USER" "$_bashrc" 2>/dev/null || true

echo ""
success "~/.bashrc actualizado. Abre una terminal nueva o ejecuta: source ~/.bashrc"

# ── 8b. Reemplazar {{USER_HOME}} en scripts de arranque ───────────────────────
info "Inyectando ruta HOME ($REAL_HOME) en scripts de arranque..."
for _script in "$INSTALL_DIR/scripts/start_agent.sh" \
               "$INSTALL_DIR/scripts/start_bridge.sh"; do
    if [[ -f "$_script" ]] && grep -q '{{USER_HOME}}' "$_script"; then
        sed -i "s|{{USER_HOME}}|$REAL_HOME|g" "$_script"
        success "  Reemplazado {{USER_HOME}} → $REAL_HOME en $(basename "$_script")"
    fi
done

# También reemplazar en templates de servicios systemd
for _tpl in "$INSTALL_DIR/services/"*.tpl; do
    if [[ -f "$_tpl" ]] && grep -q '{{USER_HOME}}' "$_tpl"; then
        sed -i "s|{{USER_HOME}}|$REAL_HOME|g" "$_tpl"
        success "  Reemplazado {{USER_HOME}} → $REAL_HOME en $(basename "$_tpl")"
    fi
done

# ── 8c. Permisos de ejecución en scripts ──────────────────────────────────────
info "Asignando permisos de ejecución a scripts..."
chmod +x "$INSTALL_DIR/scripts/"*.sh 2>/dev/null || true
chmod +x "$INSTALL_DIR/menu.sh" 2>/dev/null || true
success "  Permisos +x asignados."

# ── 8d. Enlace simbólico para el comando 'status' ─────────────────────────────
info "Creando enlace simbólico: status → $INSTALL_DIR/scripts/status.sh"
sudo ln -sf "$INSTALL_DIR/scripts/status.sh" /usr/local/bin/status
success "  Ahora puedes ejecutar 'status' desde cualquier directorio."

# ── 8e. Instalar iptables-persistent y netfilter-persistent (dependencia del firewall) ─
if ! dpkg -l iptables-persistent &>/dev/null 2>&1 || ! dpkg -l netfilter-persistent &>/dev/null 2>&1; then
    info "Instalando iptables-persistent y netfilter-persistent para persistencia del firewall..."
    sudo DEBIAN_FRONTEND=noninteractive apt-get $APT_OPTS install -y iptables-persistent netfilter-persistent -qq
    success "  iptables-persistent y netfilter-persistent instalados."
else
    info "  iptables-persistent y netfilter-persistent ya instalados."
fi

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
echo -e "  ${CYAN}# Para compilar/flashear el ESP32:${RESET}"
echo -e "  . $ESP_IDF_DIR/export.sh"
echo -e "  cd $INSTALL_DIR && ./menu.sh  # opción 3"
echo ""
echo -e "  ${CYAN}# IP de este equipo (configurar como AGENT_IP en el ESP32):${RESET}"
ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v 127.0.0.1 | \
    while read -r ip; do echo -e "    ${GREEN}$ip${RESET}"; done
echo ""
echo -e "  ${YELLOW}Nota: Los sources ya fueron añadidos al ~/.bashrc.${RESET}"
echo -e "  ${YELLOW}Abre una terminal nueva o ejecuta: source ~/.bashrc${RESET}"
echo ""
