# Proyecto micro-ROS con ESP32 y Sensor DS18B20

Este proyecto integra un sensor de temperatura DS18B20 con ESP32 usando micro-ROS para comunicarse con ROS 2.

## sensores
integracion de sensores en sistema embebido esp32

## Activar ambiente y monitor serial

- Entrar al directorio del proyecto /microRostest
- source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
- idf.py monitor

## 📋 Contenido

- [Requisitos](#requisitos)
- [Configuración del entorno](#configuración-del-entorno)
- [Activar el Monitor Serial ESP-IDF](#activar-el-monitor-serial-esp-idf)
- [Configurar ROS 2 en PC](#configurar-ros-2-en-pc)
- [Configurar micro-ROS en ESP32](#configurar-micro-ros-en-esp32)
- [Comunicación ROS 2 ↔ ESP32](#comunicación-ros-2--esp32)

---

## 🔧 Requisitos

### Hardware
- ESP32 (cualquier modelo)
- Sensor DS18B20
- Resistencia pull-up 4.7kΩ
- Cable USB

### Software
- ESP-IDF v5.5.2
- ROS 2 Jazzy (ya instalado en `/opt/ros/jazzy/`)
- micro-ROS component para ESP-IDF

---

## ⚙️ Configuración del entorno

### 1. Activar entorno ESP-IDF

En VSCode, las terminales normales NO tienen ESP-IDF configurado. Debes inicializarlo:

**Opción A: Usar terminal ESP-IDF de VSCode**
```bash
# Usa la terminal "ESP-IDF Terminal" que ya está configurada
```

**Opción B: Inicializar manualmente en cualquier terminal**
```bash
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
```

### 2. Cambiar al directorio del proyecto
```bash
cd /home/lab-ros/Documentos/Github/microRostest
```

---

## 🖥️ Activar el Monitor Serial ESP-IDF

### Método 1: Desde la terminal (recomendado)
```bash
# 1. Inicializar entorno ESP-IDF
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh

# 2. Ir al directorio del proyecto
cd /home/lab-ros/Documentos/Github/microRostest

# 3. Abrir el monitor
idf.py monitor
```

**Para salir del monitor:** `Ctrl + ]`

### Método 2: Todo en un solo comando
```bash
cd /home/lab-ros/Documentos/Github/microRostest && \
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh && \
idf.py monitor
```

### Método 3: Build, Flash y Monitor juntos
```bash
cd /home/lab-ros/Documentos/Github/microRostest && \
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh && \
idf.py build flash monitor
```

### Método 4: Desde VSCode Command Palette
1. `Ctrl + Shift + P`
2. Buscar: "ESP-IDF: Monitor Device"
3. Seleccionar el puerto serial

---

## 🤖 Configurar ROS 2 en PC

### 1. Configurar entorno ROS 2

Crea o edita `~/.bashrc` y añade:
```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Configuración de red para micro-ROS
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Luego aplica los cambios:
```bash
source ~/.bashrc
```

### 2. Instalar micro-ROS Agent

El agente actúa como puente entre el ESP32 y ROS 2:

```bash
# Instalar dependencias
sudo apt update
sudo apt install -y python3-pip

# Instalar micro-ROS agent
sudo apt install ros-jazzy-micro-ros-agent
```

### 3. Iniciar el micro-ROS Agent

**Opción A: Conexión Serial (USB)**
```bash
# Puerto serial (reemplaza /dev/ttyUSB0 por tu puerto)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Opción B: Conexión WiFi (UDP)**
```bash
# Si el ESP32 está conectado por WiFi
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 4. Verificar conexión
```bash
# En otra terminal, listar tópicos
ros2 topic list

# Deberías ver los tópicos del ESP32
ros2 topic echo /temperatura
```

---

## 🔌 Configurar micro-ROS en ESP32

### 1. Estructura del código micro-ROS

Tu código actual (`sensor_temp.c`) solo lee el sensor. Para convertirlo en nodo ROS, necesitas:

```c
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

// Publicador de temperatura
rcl_publisher_t temperature_publisher;
std_msgs__msg__Float32 temp_msg;
```

### 2. Configuración WiFi para micro-ROS (opcional)

Si quieres conectar por WiFi en lugar de USB:

```bash
# Configurar WiFi
idf.py menuconfig

# Ir a: micro-ROS Settings → WiFi Configuration
# Ingresar SSID y contraseña
```

### 3. Compilar y flashear

```bash
cd /home/lab-ros/Documentos/Github/microRostest
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh

# Limpiar (opcional)
idf.py fullclean

# Compilar
idf.py build

# Flashear al ESP32
idf.py flash

# Monitorear
idf.py monitor
```

---

## 🔄 Comunicación ROS 2 ↔ ESP32

### Flujo de comunicación

```
┌──────────┐    Serial/WiFi    ┌─────────────────┐    DDS    ┌──────────┐
│  ESP32   │ ←──────────────→  │ micro-ROS Agent │ ←───────→ │ ROS 2 PC │
│(micro-ROS)│                   │  (Puente)       │           │  (Nodes) │
└──────────┘                   └─────────────────┘           └──────────┘
```

### Ejemplo de flujo completo

**Terminal 1: micro-ROS Agent**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Terminal 2: ESP32 Monitor (opcional)**
```bash
cd /home/lab-ros/Documentos/Github/microRostest
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
idf.py monitor
```

**Terminal 3: Ver mensajes ROS 2**
```bash
source /opt/ros/jazzy/setup.bash

# Listar nodos activos
ros2 node list

# Listar tópicos
ros2 topic list

# Escuchar temperatura
ros2 topic echo /temperatura

# Ver información del tópico
ros2 topic info /temperatura

# Ver frecuencia de publicación
ros2 topic hz /temperatura
```

---

## 🐛 Troubleshooting

### Error: "idf.py: no se encontró la orden"
**Solución:** No has inicializado el entorno ESP-IDF
```bash
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
```

### Error: "CMakeLists.txt not found"
**Solución:** Estás en el directorio incorrecto
```bash
cd /home/lab-ros/Documentos/Github/microRostest
```

### Error: VSCode muestra errores en CMakeLists.txt
**Solución:** El archivo está configurado como Python en lugar de CMake
- Click en "Python" en la esquina inferior derecha
- Cambiar a "CMake"

### ESP32 no se conecta al Agent
1. Verificar que el puerto serial es correcto: `ls /dev/ttyUSB*`
2. Dar permisos: `sudo chmod 666 /dev/ttyUSB0`
3. Reiniciar el ESP32
4. Verificar que el baud rate coincide (115200)

### No aparecen tópicos en ROS 2
1. Verificar que el Agent está corriendo
2. Verificar que `ROS_LOCALHOST_ONLY=0`
3. Verificar que ambos usan el mismo `ROS_DOMAIN_ID`

---

## 📚 Recursos adicionales

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [micro-ROS for ESP-IDF](https://github.com/micro-ROS/micro_ros_espidf_component)
- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)

---

## 📁 Estructura del proyecto

```
microRostest/
├── CMakeLists.txt                    # Configuración principal del proyecto
├── sdkconfig                         # Configuración ESP-IDF
├── components/
│   └── micro_ros_espidf_component/   # Librería micro-ROS
├── main/
│   ├── CMakeLists.txt               # Configuración del componente
│   └── sensor_temp.c                # Código principal
└── README.md                        # Este archivo
```
* For a feature request or bug report, create a [GitHub issue](https://github.com/espressif/esp-idf/issues)

We will get back to you as soon as possible.
