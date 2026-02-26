# 🌊 Sistema de Monitoreo Ambiental con ROS 2 + ESP32

<div align="center">

**Última actualización:** 25 de febrero de 2026

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![ESP-IDF 5.4.1](https://img.shields.io/badge/ESP--IDF-5.4.1-green.svg)](https://docs.espressif.com/projects/esp-idf/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-WiFi%2FUDP-orange.svg)](https://micro.ros.org/)
[![MongoDB](https://img.shields.io/badge/MongoDB-Atlas-green.svg)](https://www.mongodb.com/cloud/atlas)
[![Python](https://img.shields.io/badge/Python-3.12-yellow.svg)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420.svg)](https://ubuntu.com/)

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
- 🎛️ **Menú unificado** — `menu.sh` como punto de entrada para configurar y operar

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
              │              PC — nativo               │
              │  micro_ros_agent UDP/8888               │
              │         │ DDS (FastRTPS)                │
              │  ros_sensor_node.py                     │
              │  ├── Parsea Float32MultiArray           │
              │  ├── Identifica ESP32 por MAC           │
              │  └── Guarda en MongoDB Atlas            │
              └────────────────────────────────────────┘
                                  │
              ┌───────────────────▼────────────────────┐
              │    Dashboard Streamlit Cloud            │
              │    (visualización + autenticación)      │
              └────────────────────────────────────────┘
```

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

### Requisitos

- **SO:** Ubuntu **24.04** LTS (nativo) o Raspberry Pi OS (Bookworm 64-bit)
- **RAM:** Mínimo 2 GB (4 GB recomendado para compilación)
- **Almacenamiento:** 8 GB libres
- Conexión a internet

> **Nota Raspberry Pi:** La instalación funciona en Raspberry Pi 4/5 con 2GB+ RAM. La compilación de micro-ROS puede tardar 30-60 minutos usando modo secuencial para evitar problemas de memoria.

### Pasos

1. Clona el repositorio:
   ```bash
   git clone https://github.com/Menderin/Monitoreo-de-Sensores---Implementacion-ROS.git
   cd Monitoreo-de-Sensores---Implementacion-ROS
   ```

2. Ejecuta el instalador:
   ```bash
   chmod +x install.sh && sudo ./install.sh
   ```

   El script se encarga de:
   - Añadir el usuario al grupo **dialout** (acceso a puerto serie del ESP32)
   - Instalar **ROS 2 Jazzy** via apt (incluye `std-msgs`, `ros2topic`, `rclpy`)
   - Compilar el **micro-ROS Agent** desde fuente con colcon (`~/microros_ws`)
   - Instalar **ESP-IDF v5.4.1** con paquetes Python para micro-ROS
   - Instalar dependencias Python (`pymongo`, `python-dotenv`, `certifi`, `numpy`, `pyyaml`)
   - Configurar `database/.env` con las credenciales MongoDB

3. ¡Listo! Usa el menú para operar el sistema:
   ```bash
   ./menu.sh
   ```

---

## 🎛️ Menú principal — `menu.sh`

Punto de entrada único para gestionar todo el sistema:

```bash
./menu.sh
```

| Opción | Acción |
|---|---|
| **1** | Modificar credenciales (MongoDB `database/.env` y WiFi `microros-esp/main/versions/wifi/.env`) |
| **2** | Iniciar agentes (micro-ROS Agent UDP / nodo sensores → MongoDB / motores) |
| **3** | ESP32 (compilar, flashear, monitor, Agent, calibración…) |
| **0** | Salir |

### Submenú `2) Iniciar agentes`

| Sub-opción | Acción | Cómo usar |
|---|---|---|
| **a** | Iniciar micro-ROS Agent UDP | Abrir en una terminal, Ctrl+C para detener |
| **b** | Enviar datos a MongoDB | Abrir en otra terminal (requiere agent activo) |
| **c** | Control de motores | Abrir en otra terminal (requiere agent activo) |

---

## 🚀 Uso del sistema

### Flujo típico de operación

1. **Terminal 1** — arrancar el Agent:
   ```
   ./menu.sh → 2 → a
   ```

2. **Terminal 2** — iniciar nodo de sensores:
   ```
   ./menu.sh → 2 → b
   ```

3. Verificar datos en tiempo real:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic list
   ros2 topic echo /sensor_data
   ```

4. Enviar comandos al motor:
   ```bash
   ros2 topic pub /motor_commands std_msgs/msg/String "data: 'LEFT'" --once
   ros2 topic pub /motor_commands std_msgs/msg/String "data: 'STOP'" --once
   ros2 topic pub /motor_commands std_msgs/msg/String "data: 'SPEED_SET_70'" --once
   ```

### Compilar y flashear el ESP32

Desde el menú, opción **3**, o directamente:

```bash
cd microros-esp/scripts
./microros.sh
```

---

## 🧪 Calibración del Sensor pH

```bash
source /opt/ros/jazzy/setup.bash
cd "microros-esp/scripts/utils"
python3 calibracion_ph.py
```

**Flujo:**
1. Sumergir el sensor en buffer (ej: pH 4.01)
2. Presionar **ESPACIO** → captura 10 muestras warmup + 30 reales → calcula mediana
3. Ingresar el pH real de esa solución
4. Repetir para pH 6.86 y 9.18
5. Presionar **Enter** → calcula regresión lineal → genera bloque para `config.h`

**Calibración actual (2026-02-23):**

| Buffer | Voltaje medido | Notas |
|---|---|---|
| pH 4.04 | 884 mV | Buffer estándar |
| pH 6.90 | 1703 mV | Buffer neutro |
| pH 9.23 | 2349 mV | Buffer alcalino |

```c
// config.h — calibrado 2026-02-23
#define PH_SLOPE       0.003540
#define PH_INTERCEPT   0.898120
```

---

## 📁 Estructura del Proyecto

```
sensores/
├── menu.sh                        ← ★ Punto de entrada (3 opciones)
├── install.sh                     ← Instalador automático (Ubuntu 24.04)
├── .env.example                   ← Plantilla credenciales MongoDB
│
├── database/                      ← Nodo ROS 2 + módulos MongoDB
│   ├── .env                       ← ★ Credenciales MongoDB (no commitear)
│   ├── ros_sensor_node.py         ← Suscriptor /sensor_data → MongoDB
│   └── modules/
│       ├── config.py              ← Carga database/.env, configura MongoDB
│       ├── service.py             ← SensorDBService (guardar/pingar/registrar)
│       └── crear_colecciones.py
│
├── microros-esp/                  ← Firmware ESP32 + herramientas PC
│   ├── CMakeLists.txt             ← Auto-genera wifi_config.h desde .env
│   ├── main/
│   │   ├── versions/wifi/
│   │   │   └── .env               ← ★ SSID, password, AGENT_IP, AGENT_PORT
│   │   ├── Motores/
│   │   │   └── motor_control_node.py  ← Nodo ROS 2 control de motores
│   │   └── [fuentes C del firmware]
│   └── scripts/
│       ├── microros.sh            ← Submenú ESP32 (auto-genera sdkconfig)
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
| `PH_SLOPE` | `0.003540` | Pendiente regresión pH (cal. 2026-02-23) |
| `PH_INTERCEPT` | `0.898120` | Intercepto regresión pH (cal. 2026-02-23) |
| `TEMP_OFFSET_CAL` | `-0.7` | Offset calibración temperatura |
| `PUBLISH_INTERVAL_MS` | `4000` | Publicación cada 4 segundos |
| `MOTOR_IN1_PIN` | `GPIO25` | PWM motor izquierda |
| `MOTOR_IN2_PIN` | `GPIO26` | PWM motor derecha |

---

## 🐛 Troubleshooting

### ❌ Falla compilación de micro-ROS Agent (Raspberry Pi)

**Síntoma:** Error `micro_ros_msgs` falla al compilar durante `./install.sh`

**Solución:**

1. **Limpiar workspace y reinstalar dependencias:**
   ```bash
   rm -rf ~/microros_ws/build ~/microros_ws/install
   sudo apt-get install -y python3-dev python3-setuptools build-essential
   ```

2. **Re-ejecutar instalador (ahora usa compilación secuencial):**
   ```bash
   sudo ./install.sh
   ```

3. **Si persiste el error, compilar manualmente con más verbosidad:**
   ```bash
   cd ~/microros_ws
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -y
   colcon build --symlink-install --executor sequential --parallel-workers 1 --event-handlers console_direct+
   ```

4. **Revisar logs específicos:**
   ```bash
   cat ~/microros_ws/log/latest_build/micro_ros_msgs/stderr.log
   ```

**Causas comunes:**
- Memoria insuficiente en Raspberry Pi (compilación paralela)
- Dependencias Python faltantes (`python3-dev`, `setuptools`)
- Versión incompatible de colcon o setuptools

### ❌ `rclpy._rclpy_pybind11` no importa (conflicto conda)

El menú lo resuelve automáticamente usando un subshell limpio. Si ejecutas manualmente:

```bash
# Abrir una terminal nueva sin conda activo, o:
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash
python3 database/ros_sensor_node.py
```

### ❌ ESP-IDF falla con Python 3.13 (o conflictos de colcon)

El script `microros.sh` ahora limpia automáticamente las variables de entorno ROS antes de compilar para evitar conflictos entre el colcon de ROS 2 y el colcon interno de micro_ros_espidf_component. Si aún así falla:

```bash
conda deactivate && conda deactivate
rm -rf "microros-esp/build"
source ~/esp/esp-idf/export.sh
idf.py build
```

### ❌ ESP32 no conecta al Agent WiFi

1. Verificar que `AGENT_IP` en `microros-esp/main/versions/wifi/.env` es la IP real del PC
2. Desde el menú: **3 → "Mostrar IP"** muestra la IP actual
3. Firewall: `sudo ufw allow 8888/udp`
4. Verificar en monitor serial que el ESP32 obtuvo IP
5. **Importante**: Recompilar después de cambiar `.env` (el script `microros.sh` auto-genera `sdkconfig.defaults`)

### ❌ No aparecen tópicos en ROS 2

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 topic list
```

### ❌ Permiso denegado en `/dev/ttyUSB0`

El instalador (`install.sh`) ya añade el usuario al grupo `dialout`. Si aún falla:

```bash
sudo usermod -a -G dialout $USER
# Cerrar sesión y volver a entrar, o:
newgrp dialout
```

---

## 🔧 Mejoras Técnicas Recientes

### Instalador (`install.sh`)
- ✅ **Grupo dialout automático**: Añade el usuario al grupo para acceso serie del ESP32
- ✅ **Detección inteligente ESP-IDF**: Verifica tanto el repositorio como el entorno Python
- ✅ **Paquetes micro-ROS en venv IDF**: Instala automáticamente `catkin_pkg`, `empy`, `lark`, `colcon-common-extensions`
- ✅ **Dependencias ROS 2 completas**: Incluye `std-msgs`, `ros2topic`, `ros2cli` para debugging
- ✅ **Python científico**: Instala `numpy` (para calibración) y `pyyaml` (herramientas ROS)
- ✅ **Soporte Raspberry Pi**: Compilación secuencial optimizada para ARM con manejo de errores robusto
- ✅ **Resolución automática dependencias**: Ejecuta `rosdep` antes de compilar micro-ROS

### Build ESP32 (`microros.sh` + `CMakeLists.txt`)
- ✅ **Auto-generación de credenciales**: `CMakeLists.txt` lee `.env` y genera `wifi_config.h` automáticamente
- ✅ **Sincronización sdkconfig**: `microros.sh` genera `sdkconfig.defaults` desde `.env` antes de compilar
- ✅ **Limpieza de entorno**: Desactiva variables ROS durante build para evitar conflictos colcon
- ✅ **Detección automática puerto**: Busca `/dev/ttyUSB*` y `/dev/ttyACM*` automáticamente

### Sistema de menú (`menu.sh`)
- ✅ **Source ESP-IDF inteligente**: Busca en múltiples ubicaciones (`IDF_PATH`, `~/esp/esp-idf`, versiones específicas)
- ✅ **Subshells limpios**: Ejecuta agentes ROS en `bash --norc --noprofile` para evitar conflictos conda
- ✅ **Rutas corregidas**: WiFi `.env` en `microros-esp/main/versions/wifi/.env` (no en `main/.env`)

---

## 📝 Roadmap

- [x] Sensor CWT-BL pH + temperatura
- [x] Comunicación micro-ROS WiFi/UDP
- [x] Publicación Float32MultiArray con ID por MAC
- [x] Control motor DC via ROS 2
- [x] MongoDB Atlas + Dashboard Streamlit
- [x] Herramienta calibración pH con regresión numpy
- [x] Soporte múltiples ESP32 simultáneos
- [x] Instalación automática con `install.sh` (ROS 2 + micro-ROS Agent + Python)
- [x] Menú unificado `menu.sh` (credenciales, agentes, ESP32)- [x] Auto-configuración permisos serie (grupo dialout)
- [x] Auto-generación credenciales WiFi desde .env (sin reconfigurar manualmente)
- [x] Detección y limpieza de conflictos Python/colcon en build- [ ] Alertas automáticas por valores fuera de rango
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

<div align="center">

**⭐ Si te resulta útil, dale una estrella en GitHub ⭐**

</div>
