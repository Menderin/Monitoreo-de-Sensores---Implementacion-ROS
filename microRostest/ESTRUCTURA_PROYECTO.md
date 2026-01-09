# 📁 Estructura del Proyecto microRostest

## 📂 Directorios principales

```
microRostest/
├── 📄 CMakeLists.txt              # Configuración del proyecto ESP-IDF
├── 📄 sdkconfig                   # Configuración del ESP32
│
├── 📝 README.md                   # ⭐ Documentación completa
├── 📝 QUICKSTART.md               # Guía de inicio rápido (5 min)
├── 📝 USO_NODO_ROS.md            # Guía detallada del nodo ROS
├── 📝 RESUMEN_FINAL.md           # Resumen de cambios
├── 🚀 START_HERE.sh               # ⭐ Script de inicio principal
│
├── 📁 main/                       # 💻 Código fuente del ESP32
│   ├── sensor_temp.c              # ⭐ Código principal (Nodo ROS)
│   └── CMakeLists.txt             # Config del componente
│
├── 📁 scripts/                    # 🛠️ Herramientas y utilidades
│   ├── build_and_flash.sh         # Compilar/flashear ESP32
│   ├── microros_helper.sh         # Menú interactivo completo
│   ├── install_microros_agent.sh  # Instalar agent ROS
│   ├── pc_temperature_subscriber.py # Ejemplo nodo Python
│   └── README.md                  # Documentación de scripts
│
├── 📁 components/                 # Componentes ESP-IDF
│   └── micro_ros_espidf_component/  # Librería micro-ROS
│
├── 📁 managed_components/         # Dependencias gestionadas
│   ├── espressif__ds18b20/        # Driver sensor temperatura
│   └── espressif__onewire_bus/    # Driver bus OneWire
│
└── 📁 build/                      # Archivos de compilación
```

---

## 🎯 Archivos clave para empezar

| Archivo | Propósito | Cuándo usarlo |
|---------|-----------|---------------|
| [START_HERE.sh](START_HERE.sh) | Script de bienvenida | Primera vez o cuando no sepas por dónde empezar |
| [scripts/microros_helper.sh](scripts/microros_helper.sh) | Menú completo | Desarrollo día a día |
| [main/sensor_temp.c](main/sensor_temp.c) | Código del ESP32 | Modificar funcionalidad del nodo ROS |
| [scripts/build_and_flash.sh](scripts/build_and_flash.sh) | Compilar ESP32 | Después de modificar código |

---

## 📚 Documentación

| Documento | Descripción | Nivel |
|-----------|-------------|-------|
| [README.md](README.md) | Guía completa del proyecto | 📖 Completo |
| [QUICKSTART.md](QUICKSTART.md) | Inicio rápido en 5 minutos | 🚀 Básico |
| [USO_NODO_ROS.md](USO_NODO_ROS.md) | Uso detallado como nodo ROS | 🎓 Avanzado |
| [RESUMEN_FINAL.md](RESUMEN_FINAL.md) | Resumen de transformación | 📋 Resumen |
| [scripts/README.md](scripts/README.md) | Documentación de scripts | 🛠️ Herramientas |

---

## 🔧 Código fuente

### main/sensor_temp.c
**Nodo ROS en ESP32**
- Lee sensor DS18B20 (GPIO 4)
- Publica en tópico `/temperatura` (std_msgs/Float32)
- Frecuencia: 0.5 Hz (cada 2 segundos)
- Se conecta automáticamente al micro-ROS Agent

### scripts/pc_temperature_subscriber.py
**Nodo ROS en PC (ejemplo)**
- Se suscribe a `/temperatura`
- Convierte °C → °F
- Calcula estadísticas
- Muestra alertas

---

## 🛠️ Scripts de utilidad

### Scripts de ESP32
- **build_and_flash.sh**: Compilar y flashear
- **microros_helper.sh**: Menú interactivo completo

### Scripts de ROS 2
- **install_microros_agent.sh**: Instalar agent
- **pc_temperature_subscriber.py**: Ejemplo suscriptor

### Script principal
- **START_HERE.sh**: Guía interactiva de inicio

---

## 🔄 Flujo de desarrollo

```
1. Modificar código:        main/sensor_temp.c
                            ↓
2. Compilar y flashear:     scripts/build_and_flash.sh
                            ↓
3. Iniciar agent:           ros2 run micro_ros_agent...
                            ↓
4. Probar:                  ros2 topic echo /temperatura
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
# Desde el directorio raíz del proyecto

# Compilar y flashear
./scripts/build_and_flash.sh

# Menú interactivo
./scripts/microros_helper.sh

# Iniciar agent
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# Ver datos
ros2 topic echo /temperatura

# Nodo Python ejemplo
python3 scripts/pc_temperature_subscriber.py
```

---

## 🔗 Enlaces útiles

- [ESP-IDF Docs](https://docs.espressif.com/projects/esp-idf/)
- [micro-ROS Docs](https://micro.ros.org/)
- [ROS 2 Docs](https://docs.ros.org/en/jazzy/)
- [DS18B20 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf)
