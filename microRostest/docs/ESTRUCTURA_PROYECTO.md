# 📁 Estructura del Proyecto microRostest

## 📂 Directorios principales

```
microRostest/
├── 📄 CMakeLists.txt              # Configuración del proyecto ESP-IDF
├── 📄 sdkconfig                   # Configuración del ESP32 (UART habilitado)
├── 📄 .gitignore                  # Archivos excluidos de Git
│
├── 📝 README.md                   # ⭐ Documentación completa del proyecto
│
├── 📁 docs/                       # 📚 Documentación adicional
│   ├── INICIO_RAPIDO.md           # Guía de inicio rápido (enfoque en Agent)
│   ├── ESTRUCTURA_PROYECTO.md     # Este archivo - Estructura del proyecto
│   ├── RESUMEN_FINAL.md           # Resumen de cambios y transformación
│   └── USO_NODO_ROS.md            # Guía detallada del nodo ROS
│
├── 📁 main/                       # 💻 Código fuente del ESP32
│   ├── sensor_temp.c              # ⭐ Código principal (Nodo micro-ROS)
│   ├── esp32_serial_transport.c   # Transporte serial custom
│   ├── esp32_serial_transport.h   # Header del transporte
│   ├── CMakeLists.txt             # Config del componente (define RMW_UXRCE_TRANSPORT_CUSTOM)
│   └── idf_component.yml          # Dependencias (ds18b20, onewire)
│
├── 📁 scripts/                    # 🛠️ Herramientas y utilidades
│   ├── microros.sh                # ⭐ Script unificado TODO-EN-UNO
│   ├── pc_temperature_subscriber.py # Nodo Python ejemplo (suscriptor)
│   └── README.md                  # Documentación detallada de scripts
│
├── 📁 components/                 # Componentes ESP-IDF
│   └── micro_ros_espidf_component/  # Librería micro-ROS para ESP32
│       ├── colcon.meta            # Config: transporte UART, no WiFi
│       └── micro_ros_src/         # Código fuente micro-ROS (generado)
│
├── 📁 managed_components/         # Dependencias gestionadas por IDF
│   ├── espressif__ds18b20/        # Driver sensor DS18B20
│   └── espressif__onewire_bus/    # Librería protocolo OneWire
│
└── 📁 build/                      # Archivos de compilación (ignorado en Git)
```

---

## 🎯 Archivos clave para empezar

| Archivo | Propósito | Cuándo usarlo |
|---------|-----------|---------------|
| [README.md](../README.md) | Documentación completa | Información general, instalación, arquitectura |
| [docs/INICIO_RAPIDO.md](INICIO_RAPIDO.md) | Guía rápida | Primera vez, cómo iniciar el Agent |
| [scripts/microros.sh](../scripts/microros.sh) | ⭐ Script unificado | Desarrollo día a día (build, flash, agent, monitor) |
| [main/sensor_temp.c](../main/sensor_temp.c) | Código del ESP32 | Modificar funcionalidad del nodo micro-ROS |
| [scripts/README.md](../scripts/README.md) | Docs de scripts | Referencia de comandos disponibles |

---

## 📚 Documentación

| Documento | Descripción | Nivel |
|-----------|-------------|-------|
| [README.md](../README.md) | Guía completa del proyecto | 📖 Completo |
| [docs/INICIO_RAPIDO.md](INICIO_RAPIDO.md) | Inicio rápido - Enfoque en Agent | 🚀 Básico |
| [docs/USO_NODO_ROS.md](USO_NODO_ROS.md) | Uso detallado como nodo ROS | 🎓 Avanzado |
| [docs/RESUMEN_FINAL.md](RESUMEN_FINAL.md) | Resumen de transformación | 📋 Resumen |
| [scripts/README.md](../scripts/README.md) | Documentación de scripts | 🛠️ Herramientas |

---

## 🔧 Código fuente

### main/sensor_temp.c
**Nodo micro-ROS en ESP32**
- Lee sensor DS18B20 (GPIO 4, protocolo OneWire)
- Publica en tópico `/temperatura` (std_msgs/Float32)
- Frecuencia: 0.5 Hz (cada 2 segundos)
- Usa transporte serial UART (NO WiFi)
- Se conecta automáticamente al micro-ROS Agent
- Reintentos automáticos cada 10s si Agent no disponible

### main/esp32_serial_transport.c/h
**Transporte serial custom**
- Implementa comunicación UART para micro-ROS
- Integración con esp32_serial_transport
- Configuración: 115200 baud, /dev/ttyUSB0

### scripts/pc_temperature_subscriber.py
**Nodo ROS 2 en PC (ejemplo)**
- Se suscribe a `/temperatura`
- Convierte °C → °F automáticamente
- Calcula estadísticas en tiempo real (min/max/promedio)
- Muestra alertas (🔥 alta >30°C, ❄️ baja <15°C)
- Interfaz colorizada con timestamps

---

## 🛠️ Scripts de utilidad

### microros.sh - Script Unificado TODO-EN-UNO
**El único script que necesitas** - Reemplaza a todos los anteriores

**Modos de uso:**
```bash
# Modo interactivo - Menú con 19 opciones
./microros.sh

# Modo CLI - Comandos directos
./microros.sh <comando>
```

**Categorías de funciones:**
- **ESP32 (7 opciones):** build, flash, monitor, clean, menuconfig, erase-flash, all
- **Agent (2 opciones):** agent-serial, agent-udp
- **ROS 2 (4 opciones):** topics, listen, node-info, hz
- **Instalación (3 opciones):** install-agent, check-deps, fix-permissions
- **Diagnóstico (3 opciones):** ports, test-serial, sysinfo

### pc_temperature_subscriber.py
**Ejemplo de nodo Python suscriptor**
- Suscripción al tópico `/temperatura`
- Visualización mejorada con emojis y colores
- Estadísticas en tiempo real
- Base para tus propios nodos

---

## 🔄 Flujo de desarrollo

### Desarrollo típico (3 terminales)
```
Terminal 1 (Agent):         ./microros.sh agent-serial
                            ↓
Terminal 2 (Monitor):       ./microros.sh monitor
                            ↓
Terminal 3 (Verificar):     ./microros.sh listen
```

### Modificar código
```
1. Editar código:           main/sensor_temp.c
                            ↓
2. Compilar:                ./microros.sh build
                            ↓
3. Flashear:                ./microros.sh flash
                            ↓
4. Monitorear:              ./microros.sh monitor
```

### Todo en uno (primera vez)
```
./microros.sh all          # Build + Flash + Monitor automático
```

---

## 📦 Dependencias

### ESP32
- ESP-IDF v5.5.2
- micro-ROS component
- ds18b20 (sensor)
- onewire_bus (comunicación)

### PC
- ROS 2 Jazzy
- micro-ROS Agent
- Python 3 (para scripts)

---

## 🚀 Comandos rápidos

```bash
# Desde el directorio scripts/
cd scripts/

# Menú interactivo completo (19 opciones)
./microros.sh

# Comandos directos más usados
./microros.sh agent-serial              # ★ Iniciar Agent (CRÍTICO)
./microros.sh monitor                   # Ver logs del ESP32
./microros.sh listen                    # Escuchar temperatura
./microros.sh build                     # Compilar proyecto
./microros.sh flash                     # Flashear ESP32
./microros.sh all                       # Build + Flash + Monitor

# Verificación del sistema
./microros.sh check-deps                # Verificar dependencias
./microros.sh ports                     # Ver puertos USB
./microros.sh topics                    # Listar tópicos ROS 2

# Nodo Python ejemplo
python3 pc_temperature_subscriber.py
```

---

## 🔗 Enlaces útiles

- [ESP-IDF Docs](https://docs.espressif.com/projects/esp-idf/)
- [micro-ROS Docs](https://micro.ros.org/)
- [ROS 2 Docs](https://docs.ros.org/en/jazzy/)
- [DS18B20 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf)
