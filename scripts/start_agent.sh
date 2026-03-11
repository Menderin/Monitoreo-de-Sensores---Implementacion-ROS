#!/bin/bash
# ==============================================================================
# start_agent.sh — Gateway Monitoring System
# Lanza el micro-ROS Agent (modo daemon / systemd)
# Compatible con cualquier usuario Linux (sin rutas estáticas)
# Usa rutas dinámicas: compatible con cualquier usuario Linux.
# ==============================================================================

MICROROS_WS="${MICROROS_WS:-$HOME/microros_ws}"

# Matar instancias anteriores (sin fallar si no hay ninguna)
sudo pkill -f micro_ros_agent 2>/dev/null || true
sleep 1

# Cargar entornos ROS 2 y micro-ROS
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$MICROROS_WS/install/setup.bash"

# exec: systemd controla el proceso real (PID tracking correcto)
exec "$MICROROS_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" \
    udp4 --port 8888
