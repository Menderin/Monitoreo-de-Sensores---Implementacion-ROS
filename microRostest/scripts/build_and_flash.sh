#!/bin/bash

# Script rápido para compilar y flashear el proyecto micro-ROS

BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Compilar ESP32 con micro-ROS             ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════╝${NC}"
echo ""

# Directorio del proyecto
PROJECT_DIR="/home/lab-ros/Documentos/Github/microRostest"
ESP_IDF_PATH="/home/lab-ros/esp/v5.5.2/esp-idf"

# Cambiar al directorio del proyecto
cd "$PROJECT_DIR"

# Inicializar ESP-IDF
echo -e "${YELLOW}⚙️ Inicializando entorno ESP-IDF...${NC}"
source "$ESP_IDF_PATH/export.sh"

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Error inicializando ESP-IDF${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Entorno ESP-IDF listo${NC}"
echo ""

# Detectar puerto
echo -e "${YELLOW}🔍 Buscando puerto serial...${NC}"
if [ -e /dev/ttyUSB0 ]; then
    PORT="/dev/ttyUSB0"
elif [ -e /dev/ttyUSB1 ]; then
    PORT="/dev/ttyUSB1"
elif [ -e /dev/ttyACM0 ]; then
    PORT="/dev/ttyACM0"
else
    echo -e "${YELLOW}⚠️ Puerto no detectado automáticamente${NC}"
    read -p "Ingresa el puerto manualmente (ej: /dev/ttyUSB0): " PORT
fi

echo -e "${GREEN}✓ Puerto: $PORT${NC}"
echo ""

# Menú
echo "¿Qué deseas hacer?"
echo "  1) Solo compilar (build)"
echo "  2) Compilar y flashear"
echo "  3) Compilar, flashear y monitorear"
echo "  4) Solo monitorear"
echo "  5) Limpiar y compilar desde cero"
echo ""
read -p "Selecciona [3]: " option
option=${option:-3}

case $option in
    1)
        echo -e "${BLUE}🔨 Compilando...${NC}"
        idf.py build
        ;;
    2)
        echo -e "${BLUE}🔨 Compilando...${NC}"
        idf.py build
        if [ $? -eq 0 ]; then
            echo -e "${BLUE}📤 Flasheando a $PORT...${NC}"
            idf.py -p "$PORT" flash
        fi
        ;;
    3)
        echo -e "${BLUE}🚀 Compilando, flasheando y monitoreando...${NC}"
        echo -e "${YELLOW}💡 Para salir del monitor: Ctrl + ]${NC}"
        sleep 2
        idf.py -p "$PORT" build flash monitor
        ;;
    4)
        echo -e "${BLUE}🖥️ Abriendo monitor (Ctrl + ] para salir)...${NC}"
        sleep 2
        idf.py -p "$PORT" monitor
        ;;
    5)
        echo -e "${BLUE}🧹 Limpiando proyecto...${NC}"
        idf.py fullclean
        echo -e "${BLUE}🔨 Compilando desde cero...${NC}"
        idf.py build
        ;;
    *)
        echo -e "${RED}Opción inválida${NC}"
        exit 1
        ;;
esac

echo ""
if [ $? -eq 0 ]; then
    echo -e "${GREEN}╔════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║            ✓ Completado                    ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${YELLOW}📝 Siguiente paso:${NC}"
    echo "   En otra terminal, ejecuta el micro-ROS Agent:"
    echo ""
    echo -e "${BLUE}   source /opt/ros/jazzy/setup.bash${NC}"
    echo -e "${BLUE}   ros2 run micro_ros_agent micro_ros_agent serial --dev $PORT${NC}"
    echo ""
    echo "   Y para ver los datos:"
    echo -e "${BLUE}   ros2 topic echo /temperatura${NC}"
else
    echo -e "${RED}╔════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║          ✗ Error en el proceso            ║${NC}"
    echo -e "${RED}╚════════════════════════════════════════════╝${NC}"
fi
