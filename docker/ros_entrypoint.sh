#!/bin/bash
# ==============================================================================
# ros_entrypoint.sh
# Sourcea el entorno ROS 2 antes de ejecutar cualquier comando.
# ==============================================================================
set -e

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Ejecutar el comando pasado al contenedor
exec "$@"
