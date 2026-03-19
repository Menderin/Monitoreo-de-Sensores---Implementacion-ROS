#!/bin/bash
# ==============================================================================
# start_agent.sh — Gateway Monitoring System
# Lanza el micro-ROS Agent (modo daemon / systemd)
# La variable {{USER_HOME}} es reemplazada por install.sh con la ruta real.
# Lee AGENT_PORT dinámicamente desde el .env de WiFi.
# ==============================================================================

# ── Aislamiento DDS: solo loopback (evita bucles en interfaces múltiples) ────
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

MICROROS_WS="${MICROROS_WS:-{{USER_HOME}}/microros_ws}"

# REPO_DIR: un nivel arriba de scripts/
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
WIFI_ENV="$REPO_DIR/microros-esp/main/versions/wifi/.env"

# Leer AGENT_PORT desde .env (fallback: 8888)
AGENT_PORT=8888
if [[ -f "$WIFI_ENV" ]]; then
    _p=$(grep "^AGENT_PORT=" "$WIFI_ENV" | cut -d'=' -f2 | tr -d '[:space:]' || true)
    [[ -n "$_p" ]] && AGENT_PORT="$_p"
fi

# Matar instancias anteriores (sin fallar si no hay ninguna)
pkill -f micro_ros_agent 2>/dev/null || true
sleep 1

# Cargar entornos ROS 2 y micro-ROS
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$MICROROS_WS/install/setup.bash"

# exec: systemd controla el proceso real (PID tracking correcto)
exec "$MICROROS_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" \
    udp4 --port "${AGENT_PORT:-8888}"
