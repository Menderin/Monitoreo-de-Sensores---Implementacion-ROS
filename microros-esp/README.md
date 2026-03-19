# 📡 Firmware ESP32 — micro-ROS WiFi/UDP

Firmware ESP32 con micro-ROS que publica datos de pH y temperatura en ROS 2 mediante **WiFi/UDP**, con identificación de dispositivo por dirección MAC.

**Última actualización:** 19 de marzo de 2026

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![ESP-IDF 5.4.1](https://img.shields.io/badge/ESP--IDF-5.4.1-green)](https://docs.espressif.com/projects/esp-idf/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-WiFi%2FUDP-orange)](https://micro.ros.org/)

---

## 🎯 Descripción

Firmware para ESP32 que lee el sensor dual **CWT-BL** (pH + temperatura) y publica los datos en un único tópico ROS 2 (`/sensor_data`) usando `Float32MultiArray`. Cada mensaje incluye la dirección MAC del ESP32 para identificación unívoca, lo que permite múltiples dispositivos publicando en el mismo tópico.

**Características:**
- 📡 Comunicación **WiFi/UDP** — sin cable, múltiples ESP32 simultáneos
- 🧪 Sensor **CWT-BL dual** — pH (0-14) + Temperatura (-20°C a 80°C), analógico
- 🔧 Identificación por **MAC address** — cada ESP32 tiene ID único
- 📊 Topic único `Float32MultiArray` con todos los datos
- ⚡ Publicación cada **4 segundos**
- 🎛️ Control de **motor DC** vía tópico `/motor_commands`

---

## 🏗️ Arquitectura

```
ESP32 (micro-ROS)
│
├── GPIO36 → ADC pH    ─► sensor_manager_filtered.c
├── GPIO39 → ADC Temp  ─┘  (Mediana 10 muestras, 2ms c/u)
│                              │
│                         ros_publisher.c
│                              │
│   Topic: /sensor_data        │
│   Float32MultiArray:         │
│   [0] temperatura  (°C)      │
│   [1] pH                     │
│   [2] voltage_raw_ph (mV)    │
│   [3] mac_part1              │
│   [4] mac_part2              │
│
├── GPIO25/26 → Motor DC (LEDC PWM)
│   Topic: /motor_commands (String)
│   Comandos: LEFT / RIGHT / STOP / SPEED_SET_XX
│
└── WiFi/UDP ──► micro_ros_agent (PC) puerto 8888
```

### Reconstrucción de MAC en Python

```python
def floats_to_mac(p1_f: float, p2_f: float) -> str:
    p1 = int(p1_f) & 0xFFFFFF
    p2 = int(p2_f) & 0xFFFFFF
    b = [(p1>>16)&0xFF, (p1>>8)&0xFF, p1&0xFF,
         (p2>>16)&0xFF, (p2>>8)&0xFF, p2&0xFF]
    return ''.join(f'{x:02X}' for x in b)  # → "AABBCCDDEEFF"
```

---

## 🔌 Hardware

| Componente | Modelo | Notas |
|---|---|---|
| Microcontrolador | ESP32-DevKit | Dual-Core @ 240MHz, WiFi integrado |
| Sensor Dual | CWT-BL | pH (0–14) + Temp (-20°C a 80°C), salida 0–5V |
| Driver Motor | MINI 298 | Control PWM directo en IN1/IN2 |

### Conexiones

```
Sensor CWT-BL    ESP32
─────────────    ─────
VCC (5V)  ──►   5V
GND       ──►   GND
Temp Out  ──►   GPIO39 (ADC1_CH3)
pH Out    ──►   GPIO36 (ADC1_CH0)

Driver MINI 298  ESP32
───────────────  ─────
IN1       ◄──   GPIO25 (PWM)
IN2       ◄──   GPIO26 (PWM)
```

> ⚠️ El CWT-BL emite 0–5V. Verificar que el sensor esté configurado para salida 0–3.3V o usar divisor de tensión.

---

## ⚙️ Configuración

### Archivo `.env` (WiFi / micro-ROS)

Ubicación: `main/versions/wifi/.env`

```env
WIFI_SSID=NombreDeTuRed
WIFI_PASSWORD=contraseña
AGENT_IP=10.42.0.1          # IP del PC con el micro-ROS Agent
AGENT_PORT=8888
```

> El fichero `CMakeLists.txt` lee este `.env` y genera automáticamente `wifi_config.h` en cada build.

### Parámetros principales (`include/config.h`)

| Parámetro | Valor | Descripción |
|---|---|---|
| `ADC_PH_CHANNEL` | `ADC_CHANNEL_0` (GPIO36) | Canal ADC sensor pH |
| `ADC_TEMP_CHANNEL` | `ADC_CHANNEL_3` (GPIO39) | Canal ADC temperatura |
| `PH_SLOPE` | `0.003780` | Pendiente regresión pH (cal. 2026-03-13) |
| `PH_INTERCEPT` | `1.152729` | Intercepto regresión pH (cal. 2026-03-13) |
| `TEMP_OFFSET_CAL` | `-0.7` | Offset calibración temperatura |
| `PUBLISH_INTERVAL_MS` | `4000` | Publicación cada 4 segundos |
| `MOTOR_IN1_PIN` | `GPIO25` | PWM motor dirección izquierda |
| `MOTOR_IN2_PIN` | `GPIO26` | PWM motor dirección derecha |

---

## 🚀 Compilar y Flashear

Desde el menú principal del proyecto:

```bash
cd /ruta/sensores
./menu.sh → 2
```

O directamente con el submenú de scripts:

```bash
cd microros-esp/scripts
./microros.sh
# → 2) Build + Flash + Monitor
```

### Requisitos

- **ESP-IDF v5.4.1** instalado (el instalador lo configura automáticamente)
- Credenciales WiFi configuradas en `main/versions/wifi/.env`
- ESP32 conectado por USB

---

## 🧪 Calibración del Sensor pH

```bash
source /opt/ros/jazzy/setup.bash
cd microros-esp/scripts/utils
python3 calibracion_ph.py
```

**Flujo:**
1. Seleccionar dispositivo (ESP32) a calibrar por MAC address
2. Sumergir en buffer → presionar ESPACIO → captura mediana de 30 muestras
3. Ingresar pH real (ej: 4.00, 6.86, 9.18)
4. Repetir para cada buffer (mínimo 2, ideal 3)
5. Presionar Enter → regresión lineal → bloque para pegar en `config.h`

**Calibración actual (2026-03-13):**

| Buffer | Voltaje medido |
|---|---|
| pH 4.00 | 756 mV |
| pH 6.86 | 1504 mV |
| pH 9.18 | 2127 mV |

---

## 📁 Estructura

```
microros-esp/
├── CMakeLists.txt               ← Lee .env y genera wifi_config.h automáticamente
├── main/
│   ├── versions/wifi/
│   │   ├── .env                 ← ★ SSID, password, AGENT_IP, AGENT_PORT
│   │   ├── include/
│   │   │   ├── config.h         ← Parámetros sensores, calibración, pines
│   │   │   └── motor_controller.h
│   │   └── src/
│   │       ├── ros_publisher.c  ← Publica Float32MultiArray con MAC
│   │       ├── sensor_manager_filtered.c  ← Mediana 10 muestras ADC
│   │       └── motor_controller.c
│   └── Motores/
│       └── motor_control_node.py  ← Nodo ROS 2 PC para control de motores
└── scripts/
    ├── microros.sh              ← ★ Script principal (compilar/flashear/diagnosticar)
    ├── README.md                ← Documentación de scripts
    └── utils/
        └── calibracion_ph.py   ← Herramienta calibración pH con filtro MAC
```

---

## 🐛 Troubleshooting

### ESP32 no conecta al Agent WiFi

1. Verificar que `AGENT_IP` en `.env` es la IP real del PC
2. `./microros.sh → 13` (Mostrar IP del PC)
3. Firewall: `sudo ufw allow 8888/udp`
4. Recompilar después de cambiar `.env`: `./microros.sh → 2`

### No aparecen tópicos en ROS 2

```bash
# Forzar DDS a usar solo loopback
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic list
```

### ESP-IDF falla al compilar

```bash
# Limpiar y recompilar
cd microros-esp
rm -rf build/
./scripts/microros.sh → 3   # fullclean
./scripts/microros.sh → 2   # build+flash
```

---

**Ver documentación completa del sistema:** [README.md](../README.md)