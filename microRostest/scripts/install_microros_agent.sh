#!/bin/bash

# Script de instalación de micro-ROS Agent para ROS 2 Jazzy

BLUE='\033[0;34m'
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}╔═══════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Instalador de micro-ROS Agent - Jazzy   ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════╝${NC}"
echo ""

# Verificar ROS 2 Jazzy
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo -e "${RED}✗ Error: ROS 2 Jazzy no encontrado${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ROS 2 Jazzy encontrado${NC}"
source /opt/ros/jazzy/setup.bash

echo ""
echo -e "${YELLOW}Instalando dependencias...${NC}"
sudo apt update
sudo apt install -y python3-pip ros-dev-tools

echo ""
echo -e "${YELLOW}Instalando micro-ROS Agent...${NC}"

# Verificar si ya existe
if dpkg -l | grep -q "ros-jazzy-micro-ros-agent"; then
    echo -e "${GREEN}✓ micro-ROS Agent ya está instalado${NC}"
else
    sudo apt install -y ros-jazzy-micro-ros-agent
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ micro-ROS Agent instalado correctamente${NC}"
    else
        echo -e "${YELLOW}⚠ Instalación desde apt falló. Instalando desde fuente...${NC}"
        
        # Crear workspace
        mkdir -p ~/microros_ws/src
        cd ~/microros_ws/src
        
        # Clonar repositorio
        git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
        
        cd ~/microros_ws
        source /opt/ros/jazzy/setup.bash
        
        # Instalar dependencias
        rosdep update
        rosdep install --from-paths src --ignore-src -y
        
        # Compilar
        colcon build
        
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ micro-ROS Agent compilado desde fuente${NC}"
            echo ""
            echo -e "${YELLOW}Añade esto a tu ~/.bashrc:${NC}"
            echo -e "source ~/microros_ws/install/setup.bash"
        else
            echo -e "${RED}✗ Error compilando micro-ROS Agent${NC}"
            exit 1
        fi
    fi
fi

echo ""
echo -e "${GREEN}════════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ Instalación completada${NC}"
echo -e "${GREEN}════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Para iniciar el agente:${NC}"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200"
echo ""
echo -e "${YELLOW}O usar el script helper:${NC}"
echo "  ./microros_helper.sh"
echo ""
