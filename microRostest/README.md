# 🌡️🧪 Sensor CWT-BL (pH + Temperatura) con micro-ROS

Sistema completo de monitoreo ambiental usando ESP32 + sensor CWT-BL que publica datos de pH y temperatura en ROS 2 mediante comunicación serial (UART).

**Última actualización:** 14 de enero de 2026

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.5.2-green)](https://docs.espressif.com/projects/esp-idf/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-Serial-orange)](https://micro.ros.org/)
[![MongoDB](https://img.shields.io/badge/MongoDB-Atlas-green)](https://www.mongodb.com/cloud/atlas)

## 🎯 Descripción

Integración del sensor dual CWT-BL (pH y temperatura) con ESP32 usando **micro-ROS** para publicar lecturas al ecosistema ROS 2 y almacenarlas en **MongoDB Atlas**.

**Características principales:**
- 🌡️ Lectura de temperatura del sensor CWT-BL (-20°C a 80°C, analógico)
- 🧪 Lectura de pH del sensor CWT-BL (0-14 pH, analógico)
- 📡 Comunicación serial UART (115200 baudios) entre ESP32 y PC
- 🤖 Nodo micro-ROS que publica en `/temperatura` y `/ph` (std_msgs/Float32)
- ⚡ Publicación cada 4 segundos (0.25 Hz)
- 🗄️ Almacenamiento automático en MongoDB Atlas
- 💾 Respaldo local en archivos JSON
- 🔧 Scripts de automatización incluidos

---

## 📋 Tabla de Contenido

- [Hardware Requerido](#-hardware-requerido)
- [Software Necesario](#-software-necesario)
- [Guía de Inicio Rápido](#-guía-de-inicio-rápido)
- [Arquitectura del Sistema](#-arquitectura-del-sistema)
- [Configuración MongoDB Atlas](#-configuración-mongodb-atlas)
- [Instalación Detallada](#-instalación-detallada)
- [Uso del Sistema](#-uso-del-sistema)
- [Estructura del Proyecto](#-estructura-del-proyecto)
- [Scripts Disponibles](#-scripts-disponibles)
- [Troubleshooting](#-troubleshooting)
- [Desarrollo y Modificación](#-desarrollo-y-modificación)

---

## 🔌 Hardware Requerido

### Componentes
- **ESP32** (cualquier modelo con ADC y UART)
- **Sensor CWT-BL** (pH + temperatura dual, analógico)
- **Cable USB** para conexión ESP32-PC
- **Fuente 5V** para el sensor CWT-BL

### Diagrama de Conexiones
```
Sensor CWT-BL              ESP32
━━━━━━━━━━━━━         ━━━━━━
 VCC (5V)     ────────► 5V
 GND          ────────► GND
 Temp Out     ────────► GPIO39 (ADC1_CH3)
 pH Out       ────────► GPIO36 (ADC1_CH0)
              
              USB Cable
              ┌───────► PC (Ubuntu)
```

**Nota importante:** El sensor CWT-BL requiere 5V, pero las salidas analógicas son compatibles con 3.3V del ESP32 (mediante divisor de tensión interno).

---

## 💻 Software Necesario

### En el PC (Ubuntu/Linux)
- **ROS 2 Jazzy** - Framework de robótica
- **ESP-IDF 5.5.2** - Framework de desarrollo ESP32
- **micro-ROS Agent** - Puente de comunicación ESP32 ↔ ROS 2
- **Python 3** - Para scripts auxiliares

### Versiones utilizadas
```
ROS 2:    Jazzy (instalado en /opt/ros/jazzy/)
ESP-IDF:  v5.5.2 (instalado en /home/lab-ros/esp/v5.5.2/)
Python:   3.12+
```

---

## 🚀 Guía de Inicio Rápido

### **TL;DR - Comandos Rápidos**

```bash
# 1. Posicionarse en el directorio del proyecto
cd /home/lab-ros/Documentos/Github/sensores/microRostest

# 2. Usar el script unificado (recomendado)
cd scripts
./microros.sh

# 3. Menú interactivo: Opciones recomendadas
#    - Opción 2: Flashear ESP32
#    - Opción 8: Iniciar Agent (en otra terminal)
#    - Opción 11: Escuchar /temperatura (en otra terminal)
```

### **Guía Paso a Paso**

📄 **Ver documentación completa:** [INICIO_RAPIDO.md](INICIO_RAPIDO.md)

**Resumen:**

1. **Conectar ESP32** al puerto USB del PC
2. **Flashear firmware** (si no está flasheado):
   ```bash
   cd /home/lab-ros/Documentos/Github/sensores/microRostest
   source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
   idf.py -p /dev/ttyUSB0 flash
   ```
3. **Iniciar micro-ROS Agent** (Terminal 1):
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/microros_ws/install/setup.bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   ```
4. **Ver datos de temperatura** (Terminal 2):
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic echo /temperatura
   ```

---

## 🏗️ Arquitectura del Sistema

### Diagrama de Comunicación

```
┌──────────────────────┐
│   ESP32              │
│  ┌────────────────┐  │      Serial UART
│  │  Sensor DS18B20│  │     (115200 baud)
│  │   GPIO 4       │  │    /dev/ttyUSB0
│  └────────────────┘  │         │
│  ┌────────────────┐  │         ▼
│  │ micro-ROS Node │◄─┼────────────────┐
│  │ /esp32         │  │                 │
│  │ Topic:         │  │    ┌────────────────────────┐
│  │ /temperatura   │  │    │   micro-ROS Agent      │
│  └────────────────┘  │    │   (PC - Ubuntu)        │
└──────────────────────┘    │   DDS-XRCE Bridge      │
                            └────────────────────────┘
                                      │
                                      │ DDS
                                      ▼
                            ┌────────────────────────┐
                            │   ROS 2 Ecosystem      │
                            │   - ros2 topic echo    │
                            │   - rviz2              │
                            │   - Custom nodes       │
                            └────────────────────────┘
```

### Flujo de Datos

1. **ESP32** lee temperatura del DS18B20 cada 2 segundos
2. **micro-ROS node** (ESP32) publica mensaje Float32 en `/temperatura`
3. **Serial UART** transmite datos serializados al PC (protocolo DDS-XRCE)
4. **micro-ROS Agent** (PC) deserializa y reenvía al ecosistema ROS 2
5. **Nodos ROS 2** pueden suscribirse al tópico `/temperatura`

---

## 🔧 Instalación Detallada

### Paso 1: Clonar el Repositorio

```bash
cd /home/lab-ros/Documentos/Github
git clone https://github.com/Menderin/sensores.git
cd sensores/microRostest
```

### Paso 2: Instalar micro-ROS Agent

**Opción A: Automática (recomendado)**
```bash
cd scripts
./microros.sh install-agent
```

**Opción B: Manual**
```bash
# Crear workspace
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src

# Clonar repositorios
git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

# Compilar
cd ~/microros_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Paso 3: Configurar Permisos USB

**Con el script:**
```bash
cd scripts/
./microros.sh
# Opción 16: Configurar permisos USB
```

**O manualmente:**
```bash
# Añadir usuario al grupo dialout
sudo usermod -a -G dialout $USER

# IMPORTANTE: Cerrar sesión y volver a iniciar sesión
# O temporalmente en esta sesión:
newgrp dialout

# Verificar
groups | grep dialout    # Debe aparecer 'dialout'
ls -la /dev/ttyUSB0      # Deberías tener permisos de lectura/escritura
```

> ⚠️ **Crítico:** Si no cierras sesión después de agregar el usuario al grupo dialout, los permisos NO se aplicarán y seguirás viendo "Permission denied".

### Paso 4: Compilar Firmware ESP32

```bash
cd /home/lab-ros/Documentos/Github/sensores/microRostest

# Activar entorno ESP-IDF
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh

# Compilar
idf.py build

# Flashear al ESP32 (conectado por USB)
idf.py -p /dev/ttyUSB0 flash
```

**Nota:** Si el puerto es diferente, verifica con `ls /dev/ttyUSB*`

---

## 📖 Uso del Sistema

### Iniciar el Sistema Completo

**Terminal 1: Monitor ESP32 (opcional - para ver logs)**
```bash
cd /home/lab-ros/Documentos/Github/sensores/microRostest
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
idf.py -p /dev/ttyUSB0 monitor
```
**Salida esperada:**
```
I (2981) MICRO_ROS_TEMP: 🌡️ Lectura inicial: 25.12 °C
I (2981) MICRO_ROS_TEMP: 🔍 Esperando conexión con micro-ROS Agent en PC...
```
**Salir:** `Ctrl + ]`

**Terminal 2: micro-ROS Agent (REQUERIDO)**
```bash
# Opción A: Con el script (recomendado - limpia puertos automáticamente)
cd scripts/
./microros.sh agent-serial

# Opción B: Manual
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

> 💡 **Nota:** El script limpia automáticamente conexiones previas en el puerto antes de iniciar el Agent.
**Salida esperada:**
```
[INFO] [TermiosAgentLinux]: Serial port opened
[INFO] [Root]: create_client | client_key: 0x12345678
```

**Terminal 3: Ver Temperatura**

**Opción A: Comando ROS 2 directo**
```bash
source /opt/ros/jazzy/setup.bash

# Ver todos los tópicos
ros2 topic list

# Escuchar temperatura en tiempo real
ros2 topic echo /temperatura
```
**Salida esperada:**
```
data: 25.12
---
data: 25.18
---
```

**Opción B: Nodo Python con visualización mejorada**
```bash
cd /home/lab-ros/Documentos/Github/sensores/microRostest/scripts
python3 pc_temperature_subscriber.py
```
**Salida esperada:**
```
🌡️ [15:30:45] Temp: 25.12°C (77.22°F) | Estado: Normal | #Lectura: 1
🌡️ [15:30:47] Temp: 25.18°C (77.32°F) | Estado: Normal | #Lectura: 2
📊 Estadísticas (últimas 10 lecturas): Promedio=25.15°C | Min=25.06°C | Max=25.23°C
```
**Características:**
- ✅ Conversión °C → °F automática
- ✅ Estadísticas en tiempo real
- ✅ Alertas de temperatura (🔥/❄️)
- ✅ Timestamps y contador

### Usando Scripts Auxiliares

```bash
cd /home/lab-ros/Documentos/Github/sensores/microRostest/scripts

# Menú interactivo
./microros.sh

# O comandos directos
./microros.sh agent-serial  # Iniciar Agent
./microros.sh listen        # Ver temperatura
./microros.sh topics        # Listar tópicos
```

📄 **Documentación completa de scripts:** [scripts/README.md](scripts/README.md)

---

## 📁 Estructura del Proyecto

```
microRostest/
├── 📄 README.md                    # Este archivo - Documentación principal
├── 📄 INICIO_RAPIDO.md             # Guía de inicio rápido paso a paso
├── 📄 CMakeLists.txt               # Configuración CMake principal
├── 📄 sdkconfig                    # Configuración ESP-IDF (UART habilitado)
├── 📄 .gitignore                   # Archivos excluidos de Git
│
├── 📂 main/                        # Código principal del ESP32
│   ├── sensor_temp.c               # ★ Nodo micro-ROS + lectura DS18B20
│   ├── esp32_serial_transport.c    # Transporte serial custom
│   ├── esp32_serial_transport.h    # Header del transporte
│   ├── CMakeLists.txt              # Build del componente main
│   └── idf_component.yml           # Dependencias (ds18b20, onewire)
│
├── 📂 components/                  # Componentes ESP-IDF
│   └── micro_ros_espidf_component/ # Librería micro-ROS para ESP-IDF
│       ├── colcon.meta             # Config: transporte UART, no WiFi
│       └── micro_ros_src/          # Código fuente micro-ROS (generado)
│
├── 📂 managed_components/          # Componentes gestionados por IDF
│   ├── espressif__ds18b20/         # Driver sensor DS18B20
│   └── espressif__onewire_bus/     # Librería protocolo OneWire
│
├── 📂 scripts/                     # ★ Herramientas de desarrollo
│   ├── 📄 README.md                # Documentación de scripts
│   ├── microros.sh                 # ★ Script unificado TODO-EN-UNO
│   └── pc_temperature_subscriber.py # Ejemplo nodo Python suscriptor
│
└── 📂 build/                       # Archivos de compilación (generados)
    ├── hello_world.bin             # Firmware compilado
    ├── bootloader/                 # Bootloader ESP32
    └── partition_table/            # Tabla de particiones
```

---

## 🛠️ Scripts Disponibles

### microros.sh (★ Script Unificado TODO-EN-UNO)

Script que consolida todas las funciones en una sola herramienta con menú interactivo (19 opciones) y modo CLI:

```bash
cd scripts
./microros.sh                   # Menú interactivo completo

# O comandos CLI directos:
./microros.sh build             # Compilar proyecto
./microros.sh flash             # Flashear ESP32
./microros.sh monitor           # Monitor serial
./microros.sh all               # Build + Flash + Monitor
./microros.sh agent-serial      # Iniciar Agent por serial
./microros.sh agent-udp         # Iniciar Agent por UDP
./microros.sh listen            # Escuchar /temperatura
./microros.sh topics            # Listar tópicos ROS 2
./microros.sh install-agent     # Instalar micro-ROS Agent
./microros.sh check-deps        # Verificar dependencias
./microros.sh fix-permissions   # Configurar permisos USB
./microros.sh ports             # Ver puertos seriales
./microros.sh test-serial       # Test conexión serial
./microros.sh sysinfo           # Info del sistema
```

**Documentación completa:** [scripts/README.md](scripts/README.md)

### pc_temperature_subscriber.py

Nodo Python ejemplo que se suscribe al tópico `/temperatura` con:
- ✅ Conversión automática °C → °F
- ✅ Estadísticas en tiempo real (min/max/promedio)
- ✅ Alertas de temperatura (🔥/❄️)
- ✅ Timestamps y contador de lecturas

**Instalación de dependencias (primera vez):**
```bash
cd scripts/
pip install -r requirements.txt
```

---

## ⚙️ Configuración del Proyecto

### Configuración Clave en sdkconfig

```ini
# Transporte UART habilitado (NO WiFi)
CONFIG_MICRO_ROS_ESP_UART_TRANSPORT=y
CONFIG_MICRO_ROS_ESP_NETIF_WLAN is not set

# GPIO del sensor
CONFIG_ONEWIRE_GPIO=4

# Baudrate serial
CONFIG_ESPTOOLPY_BAUD_115200B=y
```

### Modificar Configuración

```bash
# Abrir menuconfig
cd /home/lab-ros/Documentos/Github/sensores/microRostest
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
idf.py menuconfig

# Navegar a: micro-ROS Settings
# - Transport: Serial/UART (actual)
# - GPIO configuración
# - Baudrate: 115200
```

### Información del Nodo ROS

**Nombre del nodo:** `esp32`  
**Tópico publicado:** `/temperatura`  
**Tipo de mensaje:** `std_msgs/msg/Float32`  
**Frecuencia:** ~0.5 Hz (cada 2 segundos)  
**QoS:** Reliable, Volatile

---

## 🐛 Troubleshooting

### Problemas Comunes y Soluciones

#### ❌ Error: "Permission denied: /dev/ttyUSB0"

**Causa:** Usuario sin permisos para acceder al puerto serial

**Solución:**
```bash
# Añadir usuario al grupo dialout
sudo usermod -a -G dialout $USER

# Aplicar cambios (requiere logout o:)
newgrp dialout

# O dar permisos temporales
sudo chmod 666 /dev/ttyUSB0
```

#### ❌ Error: "idf.py: command not found"

**Causa:** Entorno ESP-IDF no inicializado

**Solución:**
```bash
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
```

#### ❌ Error: "No such file or directory: /dev/ttyUSB0"

**Causa:** Puerto incorrecto o ESP32 no conectado

**Solución:**
```bash
# Ver puertos disponibles
ls /dev/ttyUSB* /dev/ttyACM*

# Usar el puerto correcto
idf.py -p /dev/ttyUSB1 flash  # o ttyACM0, etc.
```

#### ❌ ESP32 no se conecta al Agent

**Diagnóstico:**
```bash
# 1. Verificar que el ESP32 está esperando Agent
cd /home/lab-ros/Documentos/Github/sensores/microRostest
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
idf.py -p /dev/ttyUSB0 monitor

# Debe mostrar:
# I (2981) MICRO_ROS_TEMP: 🔍 Esperando conexión con micro-ROS Agent...
# W (13001) MICRO_ROS_TEMP: ⏳ Esperando agente... intento 1/10

# 2. Verificar baudrate
# Debe ser 115200 tanto en ESP32 como en Agent

# 3. Reiniciar ESP32
# Presionar botón RESET físico

# 4. Matar procesos que usan el puerto
sudo fuser -k /dev/ttyUSB0
```

**Solución si persiste:**
```bash
# Borrar flash y reflashear
idf.py -p /dev/ttyUSB0 erase-flash
idf.py -p /dev/ttyUSB0 flash
```

#### ❌ No aparecen tópicos en ROS 2

**Diagnóstico:**
```bash
# Verificar que el Agent está corriendo
ps aux | grep micro_ros_agent

# Ver variables de entorno ROS
echo $ROS_DOMAIN_ID           # Debe ser 0
echo $ROS_LOCALHOST_ONLY      # Debe ser 0 o vacío
```

**Solución:**
```bash
# Configurar variables
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Reiniciar Agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

#### ❌ Error de compilación: "CMakeLists.txt not found"

**Causa:** Directorio incorrecto

**Solución:**
```bash
# Asegúrate de estar en la raíz del proyecto
cd /home/lab-ros/Documentos/Github/sensores/microRostest
pwd  # Debe mostrar: .../sensores/microRostest
```

#### ❌ Lecturas de temperatura incorrectas (85.0°C o -127.0°C)

**Causa:** Conexión incorrecta del sensor o falta resistencia pull-up

**Solución:**
1. Verificar conexiones físicas (VCC, GND, DATA)
2. Verificar resistencia pull-up de 4.7kΩ entre DATA y VCC
3. Verificar que el sensor sea DS18B20 genuine
4. Añadir delay mayor entre lecturas si el problema persiste

#### ❌ Puerto USB se desconecta constantemente

**Causa:** Cable USB defectuoso o puerto USB con problemas

**Solución:**
- Usar otro cable USB
- Conectar a otro puerto USB del PC
- Verificar alimentación: `lsusb` debe mostrar el ESP32

---

## 💻 Desarrollo y Modificación

### Modificar Frecuencia de Publicación

En [main/sensor_temp.c](main/sensor_temp.c):

```c
// Cambiar este valor (en milisegundos)
vTaskDelay(pdMS_TO_TICKS(2000));  // 2000ms = 2s (actual)
// Ejemplo para 1 segundo:
vTaskDelay(pdMS_TO_TICKS(1000));  // 1000ms = 1s
```

### Añadir Otro Sensor

1. Añadir dependencia en `main/idf_component.yml`
2. Incluir header en `sensor_temp.c`
3. Crear nuevo publisher:
   ```c
   rcl_publisher_t new_sensor_publisher;
   std_msgs__msg__Float32 new_sensor_msg;
   ```
4. Publicar en el loop principal

### Cambiar Tipo de Mensaje

Ejemplo para publicar temperatura con timestamp:

```c
// En vez de std_msgs/Float32, usar sensor_msgs/Temperature
#include <sensor_msgs/msg/temperature.h>

sensor_msgs__msg__Temperature temp_msg;
temp_msg.header.stamp.sec = (int32_t)time_seconds;
temp_msg.header.stamp.nanosec = 0;
temp_msg.temperature = temperature_value;
temp_msg.variance = 0.01;  // Varianza del sensor
```

### Workflow de Desarrollo

```bash
# 1. Modificar código
nano main/sensor_temp.c

# 2. Compilar
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
idf.py build

# 3. Flashear
idf.py -p /dev/ttyUSB0 flash

# 4. Monitorear
idf.py -p /dev/ttyUSB0 monitor

# O todo junto:
idf.py -p /dev/ttyUSB0 build flash monitor
```

---

## 📊 Especificaciones Técnicas

### Hardware
- **Microcontrolador:** ESP32 (Xtensa dual-core @ 160MHz)
- **Sensor:** DS18B20 (rango: -55°C a +125°C, resolución: 0.0625°C)
- **Comunicación:** UART (115200 baud, 8N1)
- **Protocolo sensor:** 1-Wire (OneWire)

### Software
- **Framework:** ESP-IDF 5.5.2
- **Middleware:** micro-ROS (DDS-XRCE)
- **ROS:** ROS 2 Jazzy
- **Transporte:** Serial custom (no UDP/WiFi)
- **Tamaño firmware:** ~284 KB

### Rendimiento
- **Latencia:** ~50ms (lectura sensor + serialización + transmisión)
- **Frecuencia publicación:** 0.5 Hz (configurable)
- **Consumo memoria RAM:** ~170 KB
- **Consumo memoria Flash:** ~280 KB

---

## 📚 Referencias y Recursos

### Documentación Oficial
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/) - Framework ESP32
- [micro-ROS Documentation](https://micro.ros.org/docs/) - micro-ROS oficial
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/) - ROS 2
- [DS18B20 Datasheet](https://www.maximintegrated.com/en/products/sensors/DS18B20.html) - Sensor de temperatura

### Repositorios GitHub
- [micro-ROS/micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component) - Componente micro-ROS
- [espressif/esp-idf](https://github.com/espressif/esp-idf) - ESP-IDF oficial
- [micro-ROS/micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent) - Agente micro-ROS

### Tutoriales y Guías
- [micro-ROS for ESP32](https://github.com/micro-ROS/micro_ros_espidf_component#usage) - Getting started
- [OneWire Protocol](https://www.maximintegrated.com/en/design/technical-documents/tutorials/1/1796.html) - Protocolo 1-Wire

---

## 🤝 Contribuciones

¿Encontraste un bug o quieres mejorar el proyecto?

1. Fork el repositorio
2. Crea una branch: `git checkout -b feature/nueva-funcionalidad`
3. Commit cambios: `git commit -m "Añadir nueva funcionalidad"`
4. Push: `git push origin feature/nueva-funcionalidad`
5. Abre un Pull Request en [GitHub](https://github.com/Menderin/sensores)

---

## 📝 Notas Adicionales

### Configuración del Entorno en .bashrc

Para automatizar la configuración, añade a `~/.bashrc`:

```bash
# ESP-IDF (descomenta si quieres autocargar)
# alias esp-idf='source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh'

# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# micro-ROS Agent
source ~/microros_ws/install/setup.bash

# Variables ROS
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

### Monitoreo con rviz2

Para visualizar datos en rviz2, crea un nodo intermedio que convierta Float32 a un mensaje visualizable (ej: MarkerArray).

---

## ✅ Checklist de Verificación

Antes de reportar un problema, verifica:

- [ ] ESP32 conectado por USB y aparece en `/dev/ttyUSB*`
- [ ] Usuario en grupo `dialout`: `groups | grep dialout`
- [ ] ESP-IDF environment activado: `echo $IDF_PATH`
- [ ] Firmware flasheado correctamente: sin errores en `idf.py flash`
- [ ] micro-ROS Agent instalado: `which micro_ros_agent`
- [ ] Agent corriendo: `ps aux | grep micro_ros_agent`
- [ ] Puerto correcto en Agent: `/dev/ttyUSB0` @ 115200
- [ ] ROS 2 configurado: `echo $ROS_DISTRO` → `jazzy`
- [ ] Tópico visible: `ros2 topic list | grep temperatura`

---

## 📧 Soporte

- **Issues:** [GitHub Issues](https://github.com/Menderin/sensores/issues)
- **Documentación:** [INICIO_RAPIDO.md](INICIO_RAPIDO.md) | [scripts/README.md](scripts/README.md)

---

## 📄 Licencia

Este proyecto es de código abierto. Ver [LICENSE](LICENSE) para más detalles.

---

**Última actualización:** 9 de enero de 2026  
**Versión:** 1.0 (Serial UART - Estable)