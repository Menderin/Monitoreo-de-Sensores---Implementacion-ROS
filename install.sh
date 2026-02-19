#!/usr/bin/env bash
# ==============================================================================
# install.sh — Monitor de Microalgas UCN
# Script de instalación y puesta en marcha para una PC nueva (Ubuntu 22.04/24.04)
#
# Uso:
#   chmod +x install.sh && ./install.sh
# ==============================================================================

set -euo pipefail

# ─── Colores ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${CYAN}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*"; exit 1; }
header()  { echo -e "\n${BOLD}${CYAN}══════════════════════════════════════════${RESET}"; \
            echo -e "${BOLD}${CYAN}  $*${RESET}"; \
            echo -e "${BOLD}${CYAN}══════════════════════════════════════════${RESET}"; }

# ─── Variables configurables ──────────────────────────────────────────────────
REPO_URL="${REPO_URL:-https://github.com/Menderin/Monitoreo-de-Sensores---Implementacion-ROS}"
INSTALL_DIR="${INSTALL_DIR:-$HOME/Microalgas-Monitoring-ROS}"

# ==============================================================================
# 1. Verificar Ubuntu 22.04 / 24.04
# ==============================================================================
header "1/5  Verificando sistema operativo"

if [[ ! -f /etc/os-release ]]; then
    error "No se puede determinar el sistema operativo."
fi

source /etc/os-release
OS_VERSION_ID="${VERSION_ID:-}"

if [[ "$ID" != "ubuntu" ]]; then
    error "Este script requiere Ubuntu (nativo o WSL2). Sistema detectado: $ID"
fi

if [[ "$OS_VERSION_ID" != "22.04" && "$OS_VERSION_ID" != "24.04" ]]; then
    warn "Ubuntu $OS_VERSION_ID detectado. Se recomienda 22.04 o 24.04."
    read -rp "¿Continuar de todas formas? [s/N] " answer
    [[ "$answer" =~ ^[sS]$ ]] || error "Instalación cancelada."
else
    success "Ubuntu $OS_VERSION_ID detectado. Compatible."
fi

# ==============================================================================
# 2. Instalar Docker + Docker Compose plugin
# ==============================================================================
header "2/5  Verificando Docker"

install_docker() {
    info "Instalando Docker Engine..."
    sudo apt-get update -qq
    sudo apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release

    # GPG key + repositorio oficial
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg

    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
      https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
      | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    sudo apt-get update -qq
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
                            docker-buildx-plugin docker-compose-plugin

    # Agregar usuario al grupo docker (evita sudo)
    sudo usermod -aG docker "$USER"
    success "Docker instalado. Puede ser necesario cerrar sesión para aplicar el grupo."
}

if command -v docker &>/dev/null; then
    DOCKER_VERSION=$(docker --version | awk '{print $3}' | tr -d ',')
    success "Docker $DOCKER_VERSION ya instalado."
else
    install_docker
fi

# Verificar docker compose v2
if docker compose version &>/dev/null 2>&1; then
    COMPOSE_VERSION=$(docker compose version --short 2>/dev/null || echo "v2")
    success "Docker Compose $COMPOSE_VERSION disponible."
else
    error "Docker Compose plugin no encontrado. Reinstala Docker con el plugin: docker-compose-plugin"
fi

# ==============================================================================
# 3. Clonar / verificar repositorio
# ==============================================================================
header "3/5  Preparando repositorio"

if [[ -d "$INSTALL_DIR/.git" ]]; then
    success "Repositorio encontrado en $INSTALL_DIR"
    info "Actualizando con git pull..."
    git -C "$INSTALL_DIR" pull --ff-only || warn "No se pudo hacer pull. ¿Hay cambios locales?"
else
    if [[ "$REPO_URL" == *"TU_USUARIO"* ]]; then
        warn "REPO_URL no configurada. Si ya tienes el repositorio clonado,"
        warn "ejecuta este script desde la carpeta del proyecto:"
        warn "   cd /ruta/al/proyecto && bash install.sh"
        info "O configura la URL: REPO_URL=https://github.com/... bash install.sh"
        # Intentar detectar si estamos dentro del repo
        if [[ -f "$(pwd)/docker-compose.yml" && -f "$(pwd)/.env.example" ]]; then
            INSTALL_DIR="$(pwd)"
            success "Proyecto detectado en $INSTALL_DIR"
        else
            error "No se encontró el repositorio. Configura REPO_URL o ejecuta desde el directorio del proyecto."
        fi
    else
        info "Clonando repositorio en $INSTALL_DIR ..."
        git clone "$REPO_URL" "$INSTALL_DIR"
        success "Repositorio clonado."
    fi
fi

cd "$INSTALL_DIR"

# ==============================================================================
# 4. Configurar variables de entorno (.env)
# ==============================================================================
header "4/5  Configuración del entorno"

if [[ -f "database/.env" ]]; then
    success "database/.env ya existe."
    if grep -q "^MONGO_URI=mongodb+srv" database/.env; then
        success "MONGO_URI configurado correctamente."
    else
        warn "MONGO_URI en database/.env parece no estar configurado."
        warn "Revisa el archivo database/.env antes de continuar."
    fi
else
    if [[ -f ".env.example" ]]; then
        cp .env.example database/.env
        warn "Se creó database/.env desde .env.example."
        echo ""
        echo -e "${YELLOW}  ┌─────────────────────────────────────────────────────┐${RESET}"
        echo -e "${YELLOW}  │  ACCIÓN REQUERIDA: Editar database/.env              │${RESET}"
        echo -e "${YELLOW}  │                                                       │${RESET}"
        echo -e "${YELLOW}  │  Configura al menos:                                  │${RESET}"
        echo -e "${YELLOW}  │    MONGO_URI=mongodb+srv://...                        │${RESET}"
        echo -e "${YELLOW}  │    MONGO_DB=Datos_ESP                                 │${RESET}"
        echo -e "${YELLOW}  │                                                       │${RESET}"
        echo -e "${YELLOW}  │  Comando: nano database/.env                          │${RESET}"
        echo -e "${YELLOW}  └─────────────────────────────────────────────────────┘${RESET}"
        echo ""
        read -rp "¿Deseas editar database/.env ahora? [S/n] " edit_env
        if [[ ! "$edit_env" =~ ^[nN]$ ]]; then
            "${EDITOR:-nano}" database/.env
        fi
    else
        error "No se encontró .env.example. El repositorio puede estar incompleto."
    fi
fi

# Validación final de MONGO_URI
if grep -qE "^MONGO_URI=mongodb(\+srv)?://.+:.+@" database/.env; then
    success "MONGO_URI parece válido."
else
    warn "MONGO_URI no tiene el formato esperado. Verifica database/.env antes de continuar."
fi

# ==============================================================================
# 5. Build y arranque de servicios
# ==============================================================================
header "5/5  Construyendo e iniciando servicios Docker"

echo ""
echo "Servicios que se iniciarán:"
echo "  • microros_agent  — micro-ROS Agent UDP/8888"
echo "  • ros_sensor_node — Nodo ROS 2 → MongoDB"
echo ""
read -rp "¿Iniciar servicios ahora? [S/n] " start_now
if [[ "$start_now" =~ ^[nN]$ ]]; then
    info "Puedes iniciarlos manualmente con:"
    info "   docker compose up -d"
    echo ""
    exit 0
fi

# Build
info "Construyendo imágenes Docker (puede tardar varios minutos la primera vez)..."
docker compose build

# Arranque
info "Iniciando servicios en background..."
docker compose up -d

echo ""
success "══════════════════════════════════════════════"
success "  ¡Instalación completada!"
success "══════════════════════════════════════════════"
echo ""
echo "  Comandos útiles:"
echo -e "  ${CYAN}docker compose ps${RESET}                        — Estado de los servicios"
echo -e "  ${CYAN}docker compose logs -f ros_sensor_node${RESET}   — Logs del nodo ROS"
echo -e "  ${CYAN}docker compose logs -f microros_agent${RESET}    — Logs del Agent"
echo -e "  ${CYAN}docker compose down${RESET}                      — Apagar todos los servicios"
echo ""
