# 🛠️ Scripts y Herramientas

Carpeta de utilidades para desarrollo con micro-ROS + ESP32 + ROS 2.

## 📄 Archivos

### 🎛️ microros.sh (★ SCRIPT PRINCIPAL TODO-EN-UNO)

**Script unificado que reemplaza a todos los anteriores** con interfaz mejorada, manejo robusto de errores y soporte completo para desarrollo, instalación y diagnóstico.

#### 🚀 Uso Rápido

```bash
# Menú interactivo completo
./microros.sh

# O comandos directos
./microros.sh <comando>
```

#### 📋 Menú Interactivo

```
╔═══════════════════════════════════════════════════════╗
║      🤖 micro-ROS ESP32 - Control Center 🚀          ║
╚═══════════════════════════════════════════════════════╝

  ESP32 - Desarrollo
    1)  Compilar proyecto
    2)  Flashear ESP32
    3)  Monitor serial
    4)  Build + Flash + Monitor (todo en uno)
    5)  Limpiar proyecto (fullclean)
    6)  Configuración (menuconfig)
    7)  Borrar flash completa

  micro-ROS Agent
    8)  Iniciar Agent (Serial/UART)
    9)  Iniciar Agent (UDP/WiFi)

  ROS 2 - Monitoreo
    10) Ver tópicos
    11) Escuchar /temperatura
    12) Info del nodo ESP32
    13) Frecuencia de publicación (hz)

  Instalación y Configuración
    14) Instalar micro-ROS Agent
    15) Verificar dependencias
    16) Configurar permisos USB

  Diagnóstico
    17) Ver puertos seriales
    18) Test conexión serial
    19) Info del sistema
```

#### 💻 Comandos CLI Disponibles

**ESP32 - Desarrollo:**
```bash
./microros.sh build              # Compilar proyecto
./microros.sh flash              # Flashear ESP32
./microros.sh monitor            # Monitor serial
./microros.sh all                # Build + Flash + Monitor
./microros.sh clean              # Limpiar proyecto
./microros.sh menuconfig         # Abrir menuconfig
./microros.sh erase-flash        # Borrar flash completa
```

**micro-ROS Agent:**
```bash
./microros.sh agent-serial       # Iniciar Agent por serial
./microros.sh agent-udp          # Iniciar Agent por UDP
```

**ROS 2:**
```bash
./microros.sh topics             # Listar tópicos
./microros.sh listen             # Escuchar /temperatura
./microros.sh node-info          # Info del nodo ESP32
./microros.sh hz                 # Frecuencia de publicación
```

**Instalación:**
```bash
./microros.sh install-agent      # Instalar micro-ROS Agent
./microros.sh check-deps         # Verificar dependencias
./microros.sh fix-permissions    # Configurar permisos USB
```

**Diagnóstico:**
```bash
./microros.sh ports              # Ver puertos seriales
./microros.sh test-serial        # Test conexión serial
./microros.sh sysinfo            # Info del sistema
./microros.sh help               # Ver ayuda completa
```

#### ✨ Características

- ✅ **Detección automática** de puerto USB del ESP32
- ✅ **Configuración automática** de entornos (ESP-IDF y ROS 2)
- ✅ **Verificación de dependencias** al inicio
- ✅ **Manejo inteligente** de procesos que bloquean puertos
- ✅ **Instalador incluido** para micro-ROS Agent
- ✅ **Interfaz colorizada** y mensajes claros
- ✅ **Modo CLI** para automatización y scripts
- ✅ **Diagnóstico completo** del sistema
- ✅ **Sin dependencias externas** (solo bash nativo)

#### 📝 Ejemplos de Uso

**Desarrollo típico:**
```bash
# Todo en uno (recomendado para primeras pruebas)
./microros.sh all

# O paso a paso
./microros.sh build
./microros.sh flash
./microros.sh monitor
```

**Operación diaria:**
```bash
# Terminal 1: Monitor del ESP32
./microros.sh monitor

# Terminal 2: Agent
./microros.sh agent-serial

# Terminal 3: Ver datos
./microros.sh listen
```

**Primera vez:**
```bash
# Verificar sistema
./microros.sh check-deps

# Instalar Agent
./microros.sh install-agent

# Compilar y flashear
./microros.sh all
```

**Troubleshooting:**
```bash
# Ver puertos
./microros.sh ports

# Arreglar permisos
./microros.sh fix-permissions

# Test conexión
./microros.sh test-serial

# Info completa
./microros.sh sysinfo
```


### 🐍 pc_temperature_subscriber.py
**Lector de temperatura desde ROS 2**

**Instalación de dependencias (solo primera vez):**
```bash
# Instalar dependencias Python
pip install -r requirements.txt

# O con apt (recomendado)
sudo apt install python3-yaml python3-numpy
```

**Ejecutar:**
```bash
# Asegúrate de sourcing ROS 2 primero
source /opt/ros/jazzy/setup.bash
python3 pc_temperature_subscriber.py
```

**¿Qué hace?**
- Se suscribe al tópico `/temperatura`
- Muestra la temperatura con emojis y colores
- Ideal para verificar que los datos llegan correctamente

**Ejemplo de salida:**
```
[2026-01-09 15:45:23] 🌡️  25.50 °C
[2026-01-09 15:45:25] 🌡️  25.62 °C
```

---

## 🔧 Workflows Recomendados

### Desarrollo Diario (3 Terminales)

```bash
# Terminal 1: Monitor ESP32
cd ~/Documentos/Github/sensores/microRostest/scripts
./microros.sh monitor

# Terminal 2: micro-ROS Agent
./microros.sh agent-serial

# Terminal 3: Verificar datos
./microros.sh listen
# O
python3 pc_temperature_subscriber.py
```

### Primera Configuración

```bash
# 1. Verificar sistema
./microros.sh check-deps

# 2. Instalar Agent (si no existe)
./microros.sh install-agent

# 3. Configurar permisos USB
./microros.sh fix-permissions

# 4. Compilar y flashear
./microros.sh all
```

### Modificar Código

```bash
# 1. Editar main/sensor_temp.c
nano ../main/sensor_temp.c

# 2. Compilar y flashear
./microros.sh build
./microros.sh flash

# 3. Ver output
./microros.sh monitor
```

### Cambiar Configuración ESP-IDF

```bash
# 1. Abrir menuconfig
./microros.sh menuconfig

# 2. Navegar a: Component config -> micro-ROS
# 3. Cambiar transporte (UART/UDP/TCP)

# 4. Recompilar
./microros.sh clean
./microros.sh build
./microros.sh flash
```

---

## 💡 Tips y Trucos

### Permisos USB (problema común)

```bash
# Síntoma: "Permission denied" al flashear
# Solución rápida:
./microros.sh fix-permissions

# O manualmente:
sudo usermod -a -G dialout $USER
# Luego cerrar sesión y volver a entrar
```

### Agent no encuentra ESP32

```bash
# 1. Verificar puerto
./microros.sh ports

# 2. Test conexión
./microros.sh test-serial

# 3. Si el ESP32 está en /dev/ttyUSB1 (no USB0):
# Edita microros.sh línea ~30:
ESP32_PORT="/dev/ttyUSB1"
```

### Monitor no muestra nada

```bash
# Presiona el botón de RESET en el ESP32
# O usa:
./microros.sh erase-flash
./microros.sh flash
```

### Compilación falla

```bash
# Limpiar todo y recompilar
./microros.sh clean
./microros.sh build

# Si persiste, limpiar cache:
rm -rf ../build
rm -rf ../components/micro_ros_espidf_component/micro_ros_src/{build,install}
./microros.sh build
```

### Ver logs completos del Agent

```bash
# Ejecutar Agent en modo verbose
cd ~
source microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --verbosity 10
```

---

## 📚 Referencia de Comandos

### ESP32 (microros.sh)

| Comando | Descripción | Tiempo |
|---------|-------------|--------|
| `build` | Compilar proyecto | ~30s |
| `flash` | Subir firmware al ESP32 | ~10s |
| `monitor` | Ver output serial (Ctrl+] para salir) | Interactivo |
| `all` | Build + Flash + Monitor | ~40s + Monitor |
| `clean` | Limpiar y recompilar | ~40s |
| `menuconfig` | Configuración ESP-IDF | Interactivo |
| `erase-flash` | Borrar flash completo | ~5s |

### micro-ROS Agent

| Comando | Descripción |
|---------|-------------|
| `agent-serial` | Agent por UART (/dev/ttyUSB0) |
| `agent-udp` | Agent por WiFi (UDP 8888) |

### ROS 2

| Comando | Descripción |
|---------|-------------|
| `topics` | Listar todos los tópicos |
| `listen` | Escuchar `/temperatura` |
| `node-info` | Info del nodo `/micro_ros_esp32_node` |
| `hz` | Frecuencia de publicación |

### Diagnóstico

| Comando | Descripción |
|---------|-------------|
| `ports` | Ver puertos seriales disponibles |
| `test-serial` | Test conexión con ESP32 |
| `sysinfo` | Info completa del sistema |
| `check-deps` | Verificar dependencias |

---

## 🆘 Troubleshooting Rápido

| Problema | Comando | Solución |
|----------|---------|----------|
| ESP32 no flashea | `fix-permissions` | Agregar usuario a grupo dialout |
| Monitor vacío | `monitor` + RESET físico | Presionar botón RESET del ESP32 |
| Agent no conecta | `test-serial` | Verificar puerto y baudrate |
| Build falla | `clean` → `build` | Limpiar cache y recompilar |
| Puerto ocupado | `ports` | Cerrar otros monitores/agents |
| Configuración perdida | `menuconfig` | Revisar UART transport habilitado |

---

## 📝 Notas Importantes

- **Puerto predeterminado:** `/dev/ttyUSB0` (cambiable en microros.sh)
- **Baudrate:** `115200` (debe coincidir en ESP32 y Agent)
- **Transporte:** UART/Serial (NO WiFi en esta configuración)
- **ESP-IDF:** v5.5.2 en `/home/lab-ros/esp/v5.5.2/esp-idf`
- **ROS 2:** Jazzy en `/opt/ros/jazzy`
- **Agent:** Compilado en `~/microros_ws` (no disponible via apt)

---

## 🚀 Recursos Adicionales

- [Documentación ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/)
- [micro-ROS para ESP32](https://github.com/micro-ROS/micro_ros_espidf_component)
- [ROS 2 Jazzy Docs](https://docs.ros.org/en/jazzy/)
- [Troubleshooting micro-ROS](https://micro.ros.org/docs/troubleshooting/)

---

**💬 ¿Dudas?** Revisa el [README principal](../README.md) o la [Guía de Inicio Rápido](../docs/INICIO_RAPIDO.md)
