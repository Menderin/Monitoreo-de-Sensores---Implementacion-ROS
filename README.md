# 🌊 Sistema de Monitoreo Ambiental con ROS 2 + ESP32

<div align="center">

**Última actualización:** 19 de febrero de 2026

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![ESP-IDF 5.5.2](https://img.shields.io/badge/ESP--IDF-5.5.2-green.svg)](https://docs.espressif.com/projects/esp-idf/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-WiFi%2FUDP-orange.svg)](https://micro.ros.org/)
[![MongoDB](https://img.shields.io/badge/MongoDB-Atlas-green.svg)](https://www.mongodb.com/cloud/atlas)
[![Docker](https://img.shields.io/badge/Docker-Compose-2496ED.svg)](https://docs.docker.com/compose/)
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
- 📈 **Dashboard Streamlit** — desplegado en Streamlit Cloud
- 🐳 **Stack del PC en Docker** — micro-ROS Agent + nodo ROS desplegables con un comando
- 🎛️ **Menú unificado** — `menu.sh` como punto de entrada para instalar, configurar y operar

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
└─────────────────────── WiFi/UDP ────────────────────────────────────┘
                                  │
              ┌───────────────────▼────────────────────┐
              │         PC — Docker Compose             │
              │  ┌──────────────────────────────────┐  │
              │  │  microros_agent (network_mode:host)│  │
              │  │  micro-ROS Agent UDP/8888          │  │
              │  └──────────────┬───────────────────┘  │
              │                 │ DDS (FastRTPS)        │
              │  ┌──────────────▼───────────────────┐  │
              │  │  ros_node (network_mode:host)     │  │
              │  │  ros_sensor_node.py               │  │
              │  │  ├── Parsea Float32MultiArray     │  │
              │  │  ├── Identifica ESP32 por MAC     │  │
              │  │  └── Guarda en MongoDB Atlas      │  │
              │  └──────────────────────────────────┘  │
              └────────────────────────────────────────┘
                                  │
              ┌───────────────────▼────────────────────┐
              │    Dashboard Streamlit Cloud            │
              │    (visualización + autenticación)      │
              └────────────────────────────────────────┘
```

> **`network_mode: host`** en todos los servicios Docker: necesario para que el Agent reciba UDP del ESP32 sin NAT y para que ROS 2 DDS descubra nodos via multicast.

### Topics ROS 2

| Topic | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/sensor_data` | `Float32MultiArray` | ESP32 → PC | Todos los datos del sensor + MAC |
| `/motor_commands` | `String` | PC → ESP32 | Comandos: LEFT, RIGHT, STOP, SPEED_SET_XX |

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

## 🛠️ Instalación en una PC nueva

### Opción A — Instalación automática (recomendada)

1. Clona el repositorio:
   ```bash
   git clone https://github.com/Menderin/Monitoreo-de-Sensores---Implementacion-ROS.git
   cd Monitoreo-de-Sensores---Implementacion-ROS
   ```

2. Lanza el menú principal:
   ```bash
   chmod +x menu.sh && ./menu.sh
   ```

3. Selecciona **opción 1 → Instalar sistema**. El script `install.sh` se encarga de:
   - Detectar si es Ubuntu nativo, WSL2 nativo o WSL2 + Docker Desktop
   - Instalar Docker Engine + Docker Compose plugin
   - Configurar `database/.env` (credenciales MongoDB)
   - Construir las imágenes Docker y levantar los servicios

4. Configura las credenciales desde el propio menú (**opción 3**) antes de continuar.

> **Windows:** el script detecta automáticamente WSL2 + Docker Desktop y usa
> `docker-compose.windows.yml` (bridge network + unicast DDS) en lugar del compose
> principal. Ver sección [Windows / WSL2](#-windows--wsl2) para detalles.

---

### Opción B — Instalación manual paso a paso

#### 1. Instalar Docker

```bash
sudo apt-get update && sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
    | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
sudo usermod -aG docker $USER  # cerrar sesión para aplicar
```

#### 2. Configurar credenciales MongoDB

```bash
cp .env.example database/.env
nano database/.env
# Rellenar: MONGO_URI, MONGO_DB, MONGO_COLLECTION
```

#### 3. Configurar WiFi del ESP32

```bash
cp "MicroROS - ESP/main/versions/wifi/.env.example" "MicroROS - ESP/main/versions/wifi/.env"
nano "MicroROS - ESP/main/versions/wifi/.env"
# Rellenar: WIFI_SSID, WIFI_PASSWORD, AGENT_IP (IP del PC), AGENT_PORT=8888
```

#### 4. Construir e iniciar servicios

```bash
# Linux / WSL2 nativo
docker compose up -d

# WSL2 + Docker Desktop (Windows)
docker compose -f docker-compose.windows.yml up -d

docker compose ps  # verificar que ambos servicios estén Running
```

---

## 🪟 Windows / WSL2

`network_mode: host` no funciona en Docker Desktop (corre dentro de una VM Hyper-V).
Para Windows se incluye un compose alternativo que usa bridge network:

| Plataforma | Compose a usar | Soporte |
|---|---|---|
| Ubuntu nativo | `docker-compose.yml` | ✅ Completo |
| WSL2 + Docker nativo | `docker-compose.yml` | ✅ Completo |
| WSL2 + Docker Desktop | `docker-compose.windows.yml` | ✅ Con limitaciones |
| Docker Desktop (sin WSL2) | — | ❌ No soportado |

**`docker-compose.windows.yml` diferencias:**
- Bridge network `ros_net` en lugar de `network_mode: host`
- Puerto `8888:8888/udp` mapeado al host Windows
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (FastRTPS en vez de CycloneDDS)
- `config/fastrtps_bridge.xml` — desactiva multicast DDS y usa unicast entre contenedores

**AGENT_IP para el ESP32 en WSL2:**
```bash
ip addr show eth0   # dentro de WSL2 — usar esta IP, NO la de Windows
```

**Firewall Windows** (PowerShell como administrador):
```powershell
New-NetFirewallRule -DisplayName 'microROS UDP' -Direction Inbound -Protocol UDP -LocalPort 8888 -Action Allow
```

---

## 🎛️ Menú principal — `menu.sh`

Punto de entrada único para gestionar todo el sistema:

```
./menu.sh
```

| Opción | Acción |
|---|---|
| **1** | Instalar sistema (Docker + dependencias + build + up) |
| **2** | Acciones ESP32 (compilar, flashear, monitor, Agent serial/UDP) |
| **3** | Configurar credenciales (MongoDB `database/.env` y WiFi `.env`) |
| **4** | Iniciar nodo de sensores localmente (`ros_sensor_node.py`) |
| **5** | Iniciar nodo de motores localmente (`motor_control_node.py`) |
| **6** | Gestionar servicios Docker (iniciar, detener, reiniciar, logs, rebuild) |
| **7** | Salir |

> **Opciones 4 y 5:** ejecutan los nodos ROS 2 directamente en el host (sin Docker). El menú limpia automáticamente el entorno conda si está activo, crea un `.venv/` con `--system-site-packages` e instala `pymongo`/`python-dotenv` si no están.

> **Opción 6 — Gestionar servicios Docker:**
>
> | Sub-opción | Acción | Cuándo usarla |
> |---|---|---|
> | a | Iniciar servicios | Primera vez o tras `down` |
> | b | Detener servicios | Para apagar todo |
> | c | Reiniciar nodo de sensores | Cambios en archivos `.py` (~2 seg) |
> | d | Rebuild completo | Cambios en `Dockerfile` (3-10 min) |
> | e | Logs nodo sensores | Debug / verificar conexión MongoDB |
> | f | Logs micro-ROS Agent | Debug / verificar UDP del ESP32 |
> | g | Estado general | Ver si los contenedores están Running |

---

## 🚀 Uso del sistema

### Iniciar el stack completo (Docker)

```bash
# Linux / WSL2 nativo
docker compose up -d

# WSL2 + Docker Desktop
docker compose -f docker-compose.windows.yml up -d

docker compose ps                    # ver estado
docker compose logs -f ros_node      # logs nodo ROS → MongoDB
docker compose logs -f microros_agent # logs Agent UDP
docker compose down                  # apagar
```

O desde el menú: **opción 6** (sin necesidad de conocer los comandos Docker).

### Verificar datos en tiempo real

```bash
source /opt/ros/jazzy/setup.bash

ros2 topic list
ros2 topic echo /sensor_data

# Enviar comando al motor
ros2 topic pub /motor_commands std_msgs/msg/String "data: 'LEFT'" --once
ros2 topic pub /motor_commands std_msgs/msg/String "data: 'STOP'" --once
ros2 topic pub /motor_commands std_msgs/msg/String "data: 'SPEED_SET_70'" --once
```

### Compilar y flashear el ESP32

Desde el menú, opción **2**, o directamente:

```bash
cd "MicroROS - ESP/scripts"
./microros.sh all       # Build + Flash + Monitor
./microros.sh agent-udp # Solo levantar el Agent UDP
```

---

## 🧪 Calibración del Sensor pH

```bash
source /opt/ros/jazzy/setup.bash
cd "MicroROS - ESP/scripts/utils"
python3 calibracion_ph.py
```

**Flujo:**
1. Sumergir el sensor en buffer (ej: pH 4.01)
2. Presionar **ESPACIO** → captura 10 muestras warmup + 30 reales → calcula mediana
3. Ingresar el pH real de esa solución
4. Repetir para pH 6.86 y 9.18
5. Presionar **Enter** → calcula regresión lineal → genera bloque para `config.h`

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
├── menu.sh                        ← ★ Punto de entrada único (7 opciones)
├── install.sh                     ← Instalador automático (detecta Linux/WSL2)
├── docker-compose.yml             ← Linux / WSL2 nativo (network_mode: host)
├── docker-compose.windows.yml     ← WSL2 + Docker Desktop (bridge + unicast DDS)
├── .env.example                   ← Plantilla credenciales MongoDB
├── .dockerignore
│
├── docker/
│   ├── Dockerfile.ros             ← Imagen única (ros:jazzy-ros-base)
│   └── ros_entrypoint.sh          ← Sourcea ROS 2 antes del CMD
│
├── config/
│   └── fastrtps_bridge.xml        ← Perfil DDS unicast para Docker Desktop
│
├── database/                      ← Nodo ROS 2 + módulos MongoDB
│   ├── .env                       ← ★ Credenciales MongoDB (no commitear)
│   ├── ros_sensor_node.py         ← Suscriptor /sensor_data → MongoDB
│   └── modules/
│       ├── config.py              ← Carga database/.env, configura MongoDB
│       ├── service.py             ← SensorDBService (guardar/pingar/registrar)
│       └── crear_colecciones.py
│
├── MicroROS - ESP/                ← Firmware ESP32 + herramientas PC
│   ├── CMakeLists.txt
│   ├── main/
│   │   ├── versions/wifi/.env     ← ★ SSID, password, IP del Agent
│   │   ├── Motores/
│   │   │   └── motor_control_node.py  ← Nodo ROS 2 control de motores
│   │   └── [fuentes C del firmware]
│   └── scripts/
│       ├── microros.sh            ← Submenú ESP32 (15 opciones)
│       └── utils/
│           └── calibracion_ph.py  ← Herramienta calibración + numpy
│
└── legacy/                        ← Análisis y versiones anteriores
    └── analisis/
```

---

## ⚙️ Parámetros de Configuración (`config.h`)

| Parámetro | Valor actual | Descripción |
|---|---|---|
| `ADC_PH_CHANNEL` | `ADC_CHANNEL_0` (GPIO36) | Canal ADC sensor pH |
| `ADC_TEMP_CHANNEL` | `ADC_CHANNEL_3` (GPIO39) | Canal ADC temperatura |
| `PH_SLOPE` | `0.003622` | Pendiente regresión pH |
| `PH_INTERCEPT` | `0.683614` | Intercepto regresión pH |
| `TEMP_OFFSET_CAL` | `-0.7` | Offset calibración temperatura |
| `PUBLISH_INTERVAL_MS` | `4000` | Publicación cada 4 segundos |
| `MOTOR_IN1_PIN` | `GPIO25` | PWM motor izquierda |
| `MOTOR_IN2_PIN` | `GPIO26` | PWM motor derecha |

---

## 🐛 Troubleshooting

### ❌ `rclpy._rclpy_pybind11` no importa (conflicto conda)

El menú lo resuelve automáticamente. Si ejecutas manualmente:

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
python3 database/ros_sensor_node.py
```

### ❌ ESP-IDF falla con Python 3.13

```bash
conda deactivate && conda deactivate
rm -rf "MicroROS - ESP/build"
source ~/esp/v5.5.2/esp-idf/export.sh
idf.py build
```

### ❌ ESP32 no conecta al Agent WiFi

1. Verificar que `AGENT_IP` en el `.env` del ESP32 es la IP real del PC
2. Desde el menú, opción **2 → opción 13** muestra la IP actual
3. Firewall: `sudo ufw allow 8888/udp`
4. Verificar en monitor serial que el ESP32 obtuvo IP

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

### ❌ Servicios Docker no levantan

```bash
docker compose logs          # ver error exacto
docker compose down --volumes
docker compose up -d --build # rebuild desde cero
```

### ❌ En Windows/WSL2: ESP32 no llega al Agent

1. Usar `docker-compose.windows.yml`, no el compose principal
2. `AGENT_IP` del ESP32 debe ser la IP de WSL2 (`ip addr show eth0`), no la de Windows
3. Habilitar regla de firewall en Windows para UDP 8888 (ver sección Windows/WSL2)
4. Verificar que Docker Desktop tiene acceso a la red del host habilitado

---

## 📝 Roadmap

- [x] Sensor CWT-BL pH + temperatura
- [x] Comunicación micro-ROS WiFi/UDP
- [x] Publicación Float32MultiArray con ID por MAC
- [x] Control motor DC via ROS 2
- [x] MongoDB Atlas + Dashboard Streamlit
- [x] Herramienta calibración pH con regresión numpy
- [x] Soporte múltiples ESP32 simultáneos
- [x] Stack del PC dockerizado (Agent + nodo ROS)
- [x] Instalación automática con `menu.sh` + `install.sh`
- [x] Soporte Windows via WSL2 + Docker Desktop
- [x] Gestión de servicios Docker desde el menú
- [x] Montaje en vivo de código Python (sin rebuild al modificar)
- [ ] Alertas automáticas por valores fuera de rango
- [ ] OTA updates para firmware ESP32
- [ ] Panel de control motores en Dashboard
- [ ] Exportación automática periódica a JSON

---

## 👤 Autor

**Menderin** · [@Menderin](https://github.com/Menderin)

---

## 🙏 Referencias

- [micro-ROS](https://micro.ros.org/) — Framework ROS 2 para microcontroladores
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) — Framework Espressif
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) — Robot Operating System
- [Docker Compose](https://docs.docker.com/compose/)

<div align="center">

**⭐ Si te resulta útil, dale una estrella en GitHub ⭐**

</div>
