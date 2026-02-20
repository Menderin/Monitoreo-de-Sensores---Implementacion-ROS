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

# ─── Variables configurables ──────────────────────────────────────────────────
REPO_URL="${REPO_URL:-https://github.com/Menderin/Monitoreo-de-Sensores---Implementacion-ROS}"
REPO_BRANCH="${REPO_BRANCH:-main}"
INSTALL_DIR="${INSTALL_DIR:-$REAL_HOME/Microalgas-Monitoring}"
ROS_DISTRO="jazzy"
MICROROS_WS="$REAL_HOME/microros_ws"

# ==============================================================================
# 1. Verificar Ubuntu 24.04
# ==============================================================================
header "1/6  Verificando sistema operativo"

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
header "2/6  Instalando ROS 2 Jazzy"

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
        python3-colcon-common-extensions \
        python3-rosdep

    # Inicializar rosdep
    rosdep init 2>/dev/null || true
    sudo -u "$REAL_USER" rosdep update

    # Añadir source automático al .bashrc del usuario real
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
header "3/6  Instalando micro-ROS Agent"

# Verificar si ya existe y funciona
if [[ -f "$MICROROS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "$MICROROS_WS/install/setup.bash"; set -u
    if command -v micro_ros_agent &>/dev/null || ros2 pkg list 2>/dev/null | grep -q "micro_ros_agent"; then
        success "micro-ROS Agent ya instalado en $MICROROS_WS"
    else
        warn "Workspace existe pero el Agent no está disponible — recompilando..."
        sudo -u "$REAL_USER" bash -c "source /opt/ros/jazzy/setup.bash && cd '$MICROROS_WS' && colcon build --symlink-install" 2>&1 | tail -5
        set +u; source "$MICROROS_WS/install/setup.bash"; set -u
        success "micro-ROS Agent recompilado."
    fi
else
    info "Instalando dependencias de compilación..."
    sudo apt-get update -qq
    sudo apt-get install -y --no-install-recommends \
        python3-pip \
        ros-dev-tools

    info "Creando workspace en $MICROROS_WS ..."
    sudo -u "$REAL_USER" mkdir -p "$MICROROS_WS/src"
    pushd "$MICROROS_WS/src" > /dev/null

    info "Clonando micro_ros_msgs y micro-ROS-Agent (rama jazzy)..."
    sudo -u "$REAL_USER" git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git
    sudo -u "$REAL_USER" git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

    info "Compilando (puede tardar 5-15 minutos)..."
    cd "$MICROROS_WS"
    sudo -u "$REAL_USER" bash -c "source /opt/ros/jazzy/setup.bash && cd '$MICROROS_WS' && colcon build --symlink-install"

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
# 4. Instalar dependencias Python
# ==============================================================================
header "4/6  Instalando dependencias Python"

# Usar pip con --break-system-packages (Ubuntu 24.04 PEP 668)
PIP_FLAGS="--break-system-packages --quiet"

info "Instalando pymongo, python-dotenv, certifi..."
pip3 install $PIP_FLAGS \
    "pymongo>=4.0.0" \
    "python-dotenv>=1.0.0" \
    "certifi"

# Verificar importaciones críticas
python3 -c "import pymongo; import dotenv; import certifi" \
    && success "Dependencias Python verificadas." \
    || error "Falló la verificación de dependencias Python."

# ==============================================================================
# 5. Clonar / verificar repositorio
# ==============================================================================
header "5/6  Preparando repositorio"

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
# 6. Configurar credenciales MongoDB
# ==============================================================================
header "6/6  Configuración de credenciales"

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
        cp .env.example database/.env
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
            "${EDITOR:-nano}" database/.env
        fi
    else
        error "No se encontró .env.example. El repositorio puede estar incompleto."
    fi
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
echo -e "  ${CYAN}# IP de este equipo (configurar como AGENT_IP en el ESP32):${RESET}"
ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v 127.0.0.1 | \
    while read -r ip; do echo -e "    ${GREEN}$ip${RESET}"; done
echo ""
echo -e "  ${YELLOW}Nota: Los sources ya fueron añadidos al ~/.bashrc.${RESET}"
echo -e "  ${YELLOW}Abre una terminal nueva o ejecuta: source ~/.bashrc${RESET}"
echo ""
