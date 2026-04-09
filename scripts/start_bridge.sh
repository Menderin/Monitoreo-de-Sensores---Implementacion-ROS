#!/bin/bash
# ==============================================================================
# start_bridge.sh — Gateway Monitoring System
# Lanza el Bridge ROS 2 → MongoDB (modo daemon / systemd)
# La variable {{USER_HOME}} es reemplazada por install.sh con la ruta real.
# ==============================================================================

# ── Aislamiento DDS: solo loopback (evita bucles en interfaces múltiples) ────
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

MICROROS_WS="${MICROROS_WS:-{{USER_HOME}}/microros_ws}"

# REPO_DIR: un nivel arriba de este script (scripts/../)
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

# Cargar entornos ROS 2 y micro-ROS
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$MICROROS_WS/install/setup.bash"

# Entrar al directorio database para que .env sea localizable por python-dotenv
cd "$REPO_DIR/database"

# Verificar que .env existe y no está vacío antes de lanzar el bridge
if [[ ! -f ".env" ]]; then
    echo "[ERROR] Archivo .env no encontrado en $REPO_DIR/database/" >&2
    echo "[ERROR] Crea database/.env con MONGO_URI y vuelve a intentarlo." >&2
    echo "[ERROR] Puedes copiarlo desde: cp $REPO_DIR/.env.example $REPO_DIR/database/.env" >&2
    exit 1
fi

if ! grep -qE "^MONGO_URI=mongodb(\+srv)?://.+" .env; then
    echo "[WARN] MONGO_URI parece vacío o incorrecto en database/.env" >&2
    echo "[WARN] El bridge puede fallar al conectar con MongoDB Atlas." >&2
fi

# exec: systemd controla el proceso real
exec /usr/bin/python3 ros_sensor_node.py
