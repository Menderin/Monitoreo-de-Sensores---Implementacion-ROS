# 🌊 Sistema de Monitoreo Ambiental con ROS 2 + ESP32

<div align="center">

**Última actualización:** 19 de febrero de 2026

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![ESP-IDF 5.5.2](https://img.shields.io/badge/ESP--IDF-5.5.2-green.svg)](https://docs.espressif.com/projects/esp-idf/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-WiFi%2FUDP-orange.svg)](https://micro.ros.org/)
[![MongoDB](https://img.shields.io/badge/MongoDB-Atlas-green.svg)](https://www.mongodb.com/cloud/atlas)
[![Python](https://img.shields.io/badge/Python-3.12-yellow.svg)](https://www.python.org/)

**Sistema IoT de monitoreo ambiental: ESP32 + ROS 2 + micro-ROS (WiFi/UDP) + MongoDB**

</div>

---

## 📖 Descripción

Sistema completo de monitoreo ambiental con **sensor dual CWT-BL** (pH y temperatura) usando **ESP32** y **micro-ROS** sobre **WiFi/UDP**. Los datos se publican como nodos ROS 2, se almacenan en **MongoDB Atlas** y se visualizan en un **dashboard Streamlit**. Incluye control de motor DC via ROS 2.

### ✨ Características

- 🤖 **micro-ROS sobre WiFi/UDP** — sin cable serial, múltiples ESP32 simultáneos
- 🧪 **Sensor CWT-BL dual** — pH (0–14) + Temperatura (-20°C a 80°C), analógico
- 📊 **Topic único `Float32MultiArray`** — publica `[temp, pH, voltage_raw, mac_part1, mac_part2]`
- 🔧 **Identificación por MAC** — cada ESP32 tiene ID único en la red
- 🏠 **Control de motor DC** — suscriptor ROS 2 para comandos STOP/LEFT/RIGHT/SPEED
- ☁️ **MongoDB Atlas** — almacenamiento automático con timestamps
- 📈 **Dashboard Streamlit** — visualización en tiempo real con autenticación
- 🧪 **Herramienta de calibración pH** — captura medianas y calcula regresión lineal con numpy
- 📡 **Filtrado de ruido ADC** — mediana de 10 muestras por lectura

---

## 🏗️ Arquitectura del Sistema

```
┌──────────────────────────────────────────────────────────────────────┐
│                          ESP32 (micro-ROS)                           │
│                                                                      │
│  GPIO36 ──► ADC pH ──┐                                               │
│                       ├──► sensor_manager_filtered.c                 │
│  GPIO39 ──► ADC Temp ─┘    (Mediana 10 muestras)                    │
│                                      │                               │
│              ros_publisher.c ◄────────┘                              │
│  Topic: /sensor_data                                                 │
│  Msg:   Float32MultiArray                                            │
│  [0]=temp [1]=pH [2]=voltage_raw [3]=mac1 [4]=mac2                  │
│                                                                      │
│  GPIO25/26 ──► Motor DC (LEDC PWM)  ◄── /motor_commands (String)    │
└─────────────────────────── WiFi/UDP ────────────────────────────────┘
                                  │
                    ┌─────────────▼──────────────┐
                    │    micro-ROS Agent (PC)     │
                    │  UDP port 8888              │
                    └─────────────┬──────────────┘
                                  │ DDS
                    ┌─────────────▼──────────────┐
                    │      ROS 2 Jazzy (PC)       │
                    │                             │
                    │  ros_sensor_node.py         │
                    │  ├── Parsea Float32MultiArray│
                    │  ├── Identifica ESP32 por MAC│
                    │  └── Guarda en MongoDB Atlas │
                    └─────────────┬──────────────┘
                                  │
                    ┌─────────────▼──────────────┐
                    │    Dashboard Streamlit      │
                    │    (visualización + auth)   │
                    └────────────────────────────┘
```

### Topics ROS 2

| Topic | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/sensor_data` | `Float32MultiArray` | ESP32 → PC | Todos los datos del sensor + MAC |
| `/motor_commands` | `String` | PC → ESP32 | Comandos: LEFT, RIGHT, STOP, SPEED_SET_XX |

### Formato del mensaje `/sensor_data`

```
data[0] = temperatura (°C)
data[1] = pH (calibrado)
data[2] = voltage_raw_ph (mV)   ← usado para recalibración
data[3] = MAC[0:2] como float   ← identificador único ESP32
data[4] = MAC[3:5] como float
```

---

## 🔌 Hardware

### Componentes

| Componente | Modelo | Notas |
|---|---|---|
| Microcontrolador | ESP32-DevKit | Dual-Core @ 240MHz, WiFi integrado |
| Sensor Dual | CWT-BL | pH (0–14) + Temp (-20°C a 80°C), salida 0–5V |
| Driver Motor | MINI 298 | Control PWM directo en IN1/IN2 |
| Motor DC | --- | Alimentado por MINI 298 |

### Conexiones ESP32

```
Sensor CWT-BL              ESP32
─────────────         ─────────────
VCC (5V)    ────────► 5V
GND         ────────► GND
Temp Out    ────────► GPIO39 (ADC1_CH3)   [0–3.3V]
pH Out      ────────► GPIO36 (ADC1_CH0)   [0–3.3V]

Driver MINI 298            ESP32
───────────────       ─────────────
IN1         ◄────────  GPIO25  (PWM - dirección izquierda)
IN2         ◄────────  GPIO26  (PWM - dirección derecha)
```

> **Nota:** El sensor CWT-BL emite 0–5V. El ESP32 tolera máx 3.3V en entradas ADC. Verificar divisor de tensión o que el sensor esté configurado para salida 0–3.3V.

---

## 🛠️ Instalación

### 1. Prerrequisitos del sistema

```bash
# Ubuntu 24.04 LTS recomendado
# Python 3.12 (del sistema, NO usar conda/venv para ROS/ESP-IDF)
```

### 2. Instalar ROS 2 Jazzy

```bash
# Configurar repositorios
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar
sudo apt update && sudo apt install -y ros-jazzy-desktop

# Activar en sesión
source /opt/ros/jazzy/setup.bash
```

### 3. Instalar micro-ROS Agent

```bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

cd ~/microros_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 4. Instalar ESP-IDF 5.5.2

```bash
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git v5.5.2/esp-idf
cd v5.5.2/esp-idf
./install.sh esp32

# Activar entorno (siempre antes de compilar)
# ⚠️ IMPORTANTE: desactivar conda/venv antes de hacer esto
conda deactivate  # si usás conda
source ~/esp/v5.5.2/esp-idf/export.sh
```

> **⚠️ Conflicto de Python:** ESP-IDF 5.5.2 requiere **Python 3.12**. Si tenés Miniconda/Anaconda activo, el `python3` del sistema queda oculto por Python 3.13+ de conda. Siempre ejecutar `conda deactivate` antes de `source export.sh`.

### 5. Clonar y configurar proyecto

```bash
git clone https://github.com/Menderin/sensores.git
cd sensores/microRostest
```

### 6. Configurar credenciales WiFi

```bash
cd scripts
./microros.sh edit-env   # Opción 12 del menú
# Completar: WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT
./microros.sh gen-wifi   # Genera wifi_config.h
```

### 7. Compilar y flashear ESP32

```bash
# ⚠️ Desactivar conda antes
conda deactivate && conda deactivate

cd microRostest/scripts
./microros.sh all        # Build + Flash + Monitor (opción 2)
```

### 8. Instalar dependencias Python (dashboard + nodo ROS)

```bash
cd sensores
pip install -r database/requirements.txt
# o
pip install pymongo python-dotenv streamlit numpy pandas matplotlib
```

---

## 🚀 Uso del Sistema

### Iniciar el stack completo (3 terminales)

**Terminal 1 — micro-ROS Agent (WiFi/UDP)**
```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Terminal 2 — Nodo ROS → MongoDB**
```bash
source /opt/ros/jazzy/setup.bash
cd sensores/database
python3 ros_sensor_node.py
```

**Terminal 3 — Dashboard Streamlit**
```bash
cd sensores/database
streamlit run monitoreo_vivo.py
```

Con el script unificado (alternativa):
```bash
cd microRostest/scripts
./microros.sh           # Menú interactivo
./microros.sh agent-udp # Agent WiFi directo
```

### Verificar datos en tiempo real

```bash
source /opt/ros/jazzy/setup.bash

# Ver todos los tópicos
ros2 topic list

# Ver datos del sensor
ros2 topic echo /sensor_data

# Enviar comando al motor
ros2 topic pub /motor_commands std_msgs/msg/String "data: 'LEFT'" --once
ros2 topic pub /motor_commands std_msgs/msg/String "data: 'STOP'" --once
ros2 topic pub /motor_commands std_msgs/msg/String "data: 'SPEED_SET_70'" --once
```

---

## 🧪 Calibración del Sensor pH

El sistema incluye una herramienta interactiva para recalibrar el sensor en campo:

```bash
source /opt/ros/jazzy/setup.bash
cd microRostest/scripts/utils
python3 calibracion_ph.py
```

**Flujo de calibración:**
1. Sumerge el sensor en buffer (ej: pH 4.01)
2. Presiona **ESPACIO** → captura 10 muestras warmup + 30 reales → calcula mediana
3. Ingresa el pH real de esa solución
4. Repite para pH 6.86 y 9.18
5. Presiona **Enter** → calcula regresión lineal con numpy → genera bloque para `config.h`

**Calibración actual (2026-02-19):**

| Buffer | Voltaje medido | Error |
|---|---|---|
| pH 4.01 | 915 mV | +0.012 |
| pH 6.86 | 1713 mV | −0.028 |
| pH 9.18 | 2342 mV | +0.016 |

```c
// config.h — calibrado 2026-02-19, R² = 0.999912
#define PH_SLOPE       0.003622
#define PH_INTERCEPT   0.683614
```

---

## 📁 Estructura del Proyecto

```
sensores/
├── README.md                          ← Este archivo
│
├── microRostest/                      ← Firmware ESP32 + herramientas
│   ├── CMakeLists.txt
│   ├── main/
│   │   └── versions/wifi/
│   │       ├── src/
│   │       │   ├── main.c                    # Entry point, tareas FreeRTOS
│   │       │   ├── sensor_manager_filtered.c # Lectura ADC con mediana
│   │       │   ├── ros_publisher.c           # Publicador Float32MultiArray + sub motor
│   │       │   └── motor_controller.c        # Control PWM motor DC
│   │       └── include/
│   │           ├── config.h                  # ★ Calibración, pines, parámetros
│   │           ├── sensor_manager.h
│   │           ├── ros_publisher.h
│   │           └── motor_controller.h
│   │
│   ├── scripts/
│   │   ├── microros.sh                # ★ Script unificado (15 opciones)
│   │   ├── sensor_to_mongodb.py       # Alternativa directa a MongoDB
│   │   └── utils/
│   │       └── calibracion_ph.py      # ★ Herramienta calibración pH + numpy
│   │
│   └── docs/
│       ├── README_ENV.md              # Configuración .env WiFi
│       └── README_MONGODB.md          # Configuración MongoDB Atlas
│
├── database/                          ← Nodo ROS + Dashboard
│   ├── ros_sensor_node.py             # Suscriptor ROS → MongoDB
│   ├── monitoreo_vivo.py              # Dashboard Streamlit
│   └── requirements.txt
│
└── analisis/                          ← Scripts de análisis de datos
    ├── scripts/
    │   ├── analisis_temp_ph.py
    │   └── analisis_temp_ph_3Dias.py
    └── images/
```

---

## ⚙️ Parámetros de Configuración (`config.h`)

| Parámetro | Valor actual | Descripción |
|---|---|---|
| `ADC_PH_CHANNEL` | `ADC_CHANNEL_0` (GPIO36) | Canal ADC sensor pH |
| `ADC_TEMP_CHANNEL` | `ADC_CHANNEL_3` (GPIO39) | Canal ADC temperatura |
| `ADC_ATTEN` | `ADC_ATTEN_DB_12` | Rango 0–3.3V |
| `PH_SLOPE` | `0.003622` | Pendiente regresión pH |
| `PH_INTERCEPT` | `0.683614` | Intercepto regresión pH |
| `TEMP_OFFSET_CAL` | `-0.7` | Offset calibración temperatura |
| `PUBLISH_INTERVAL_MS` | `4000` | Publicación cada 4 segundos |
| `MOTOR_IN1_PIN` | `GPIO25` | PWM motor izquierda |
| `MOTOR_IN2_PIN` | `GPIO26` | PWM motor derecha |

---

## 🐛 Troubleshooting

### ❌ ESP-IDF falla con "Python 3.13 vs 3.12"

```bash
conda deactivate && conda deactivate
rm -rf microRostest/build
source ~/esp/v5.5.2/esp-idf/export.sh
idf.py build
```

### ❌ `rclpy` no importa en scripts Python

```bash
# NO usar conda ni venv para scripts ROS
conda deactivate
source /opt/ros/jazzy/setup.bash
python3 mi_script.py
```

### ❌ ESP32 no conecta al Agent WiFi

1. Verificar que `AGENT_IP` en `.env` es la IP real del PC (`./microros.sh show-ip`)
2. Verificar que el Agent UDP está corriendo en el puerto correcto
3. Desde el monitor serial (`./microros.sh monitor`) verificar que el ESP32 obtiene IP
4. Firewall: `sudo ufw allow 8888/udp`

### ❌ No aparecen tópicos en ROS 2

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 topic list
```

### ❌ Permiso denegado en `/dev/ttyUSB0`

```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

---

## 📝 Roadmap

- [x] Sensor CWT-BL pH + temperatura
- [x] Comunicación micro-ROS WiFi/UDP
- [x] Publicación Float32MultiArray con ID por MAC
- [x] Control motor DC via ROS 2
- [x] MongoDB Atlas + Dashboard Streamlit
- [x] Herramienta calibración pH con regresión numpy
- [x] Soporte múltiples ESP32 simultáneos
- [ ] Alertas automáticas por valores fuera de rango
- [ ] OTA updates para firmware ESP32
- [ ] Panel de control motores en Dashboard
- [ ] Exportación automática periódica a JSON

---

## 👤 Autor

**Menderin** · [@Menderin](https://github.com/Menderin) · [github.com/Menderin/sensores](https://github.com/Menderin/sensores)

---

## 🙏 Referencias

- [micro-ROS](https://micro.ros.org/) — Framework ROS 2 para microcontroladores
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) — Framework Espressif
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) — Robot Operating System

<div align="center">

**⭐ Si te resulta útil, dale una estrella en GitHub ⭐**

</div>
