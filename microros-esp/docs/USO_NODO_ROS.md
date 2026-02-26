# 🎯 Guía Rápida: ESP32 como Nodo ROS

**Última actualización:** 14 de enero de 2026

## ✅ El código ya está listo

Tu archivo [sensor_tempV3.c](../main/sensor_tempV3.c) ahora es un **nodo ROS completo** que:

✓ Lee pH del sensor CWT-BL (rango 0-14)
✓ Lee temperatura del sensor CWT-BL (rango -20 a 80°C)  
✓ Se conecta automáticamente al micro-ROS Agent en el PC  
✓ Publica datos en los tópicos `/ph` y `/temperatura` cada 4 segundos  
✓ Funciona como un nodo ROS estándar  

---

## 🚀 Cómo usarlo (3 terminales)

### 📟 Terminal 1: Compilar y flashear ESP32

**Opción A: Script unificado (recomendado)**
```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
./microros.sh all          # Build + Flash + Monitor automático
```

**Opción B: Paso a paso**
```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
./microros.sh build        # Compilar
./microros.sh flash        # Flashear
./microros.sh monitor      # Ver logs
```

**Opción C: Comandos manuales**
```bash
cd ~/Documentos/Github/sensores/microRostest
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
idf.py build flash monitor
```

**Lo que verás:**
```
🔧 Configurando sensor DS18B20...
✅ Sensor DS18B20 encontrado y listo
🌡️ Lectura inicial: 25.31 °C
🔍 Esperando conexión con micro-ROS Agent en PC...
```

*(Aún no se conectará hasta que inicies el Agent en el PC)*

---

### 🌉 Terminal 2: Iniciar micro-ROS Agent (Puente PC ↔ ESP32)

**Opción A: Script unificado (recomendado)**
```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
./microros.sh agent-serial
```

**Opción B: Comando manual**
```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Lo que verás en Terminal 1 (ESP32):**
```
✅ Conectado al micro-ROS Agent!
✅ Nodo ROS creado: 'esp32_temp_sensor'
✅ Publicador creado en tópico: /temperatura
🚀 Sistema iniciado. Publicando temperatura cada 2 segundos...
📡 Publicado en ROS: 25.31 °C
📡 Publicado en ROS: 25.25 °C
```

---

### 📊 Terminal 3: Ver datos en ROS 2 (PC)

**Opción A: Script unificado (recomendado)**
```bash
cd ~/Documentos/Github/sensores/microRostest/scripts

# Ver todos los tópicos
./microros.sh topics

# Escuchar temperatura en tiempo real
./microros.sh listen

# Ver info del nodo
./microros.sh node-info

# Ver frecuencia de publicación
./microros.sh hz
```

**Opción B: Comandos ROS manuales**
```bash
# Configurar ROS
source /opt/ros/jazzy/setup.bash

# Ver todos los nodos
ros2 node list
# Salida: /micro_ros_esp32_node

# Ver todos los tópicos
ros2 topic list
# Salida: /temperatura, /parameter_events, /rosout

# Escuchar temperatura en tiempo real
ros2 topic echo /temperatura
# Salida:
# data: 25.31
# ---
# data: 25.25
# ---
# data: 25.37
# ---

# Ver información del tópico
ros2 topic info /temperatura
# Salida:
# Type: std_msgs/msg/Float32
# Publisher count: 1
# Subscription count: 1

# Ver frecuencia de publicación
ros2 topic hz /temperatura
# Salida: average rate: 0.500
#         min: 2.000s max: 2.000s
```

---

## 📈 Visualizar datos

### Opción 1: rqt_plot (gráfica en tiempo real)
```bash
ros2 run rqt_plot rqt_plot
# En la interfaz, añadir: /temperatura/data
```

### Opción 2: rqt_graph (ver conexiones)
```bash
ros2 run rqt_graph rqt_graph
```

### Opción 3: Nodo Python con estadísticas (incluido en el proyecto)

**Primera vez - Instalar dependencias:**
```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
pip install -r requirements.txt
# O: sudo apt install python3-yaml python3-numpy
```

**Ejecutar:**
```bash
python3 pc_temperature_subscriber.py
```

**Salida:**
```
🌡️ [15:30:45] Temp: 25.12°C (77.22°F) | Estado: Normal | #Lectura: 1
🌡️ [15:30:47] Temp: 25.18°C (77.32°F) | Estado: Normal | #Lectura: 2
📊 Estadísticas (últimas 10 lecturas): Promedio=25.15°C | Min=25.06°C | Max=25.23°C
```

**Características:**
- ✅ Conversión automática °C → °F
- ✅ Estadísticas en tiempo real (min/max/promedio)
- ✅ Alertas de temperatura (🔥 alta >30°C, ❄️ baja <15°C)
- ✅ Timestamps y contador de lecturas

**Para crear tu propio nodo:** Ver código en [scripts/pc_temperature_subscriber.py](../scripts/pc_temperature_subscriber.py)

---

## 🔧 Arquitectura del sistema

```
┌─────────────────┐        USB Serial         ┌──────────────────┐
│     ESP32       │◄────────────────────────►│  micro-ROS Agent │
│  (Nodo ROS)     │    115200 baud            │   (Puente)       │
│                 │                           │                  │
│ ┌─────────────┐ │                           │                  │
│ │  DS18B20    │ │                           │                  │
│ │  Sensor     │ │                           │                  │
│ │  25.31°C    │ │                           │                  │
│ └─────────────┘ │                           │                  │
│                 │                           │                  │
│ Publica:        │                           │                  │
│ /temperatura    │                           │                  │
└─────────────────┘                           └──────────────────┘
                                                       │
                                                   DDS/UDP
                                                       │
                                                       ▼
                                              ┌──────────────────┐
                                              │   PC Linux       │
                                              │   ROS 2 Jazzy    │
                                              │                  │
                                              │ • Subscribers    │
                                              │ • rqt_plot       │
                                              │ • rviz2          │
                                              │ • Tus nodos      │
                                              └──────────────────┘
```

---

## 🎛️ Configuración avanzada

### Cambiar frecuencia de publicación

En [sensor_temp.c](main/sensor_temp.c), línea ~112:
```c
const unsigned int timer_timeout = 2000; // milisegundos (2 segundos)
```

Cambiar a:
- `1000` = 1 segundo (1 Hz)
- `500` = 0.5 segundos (2 Hz)
- `5000` = 5 segundos (0.2 Hz)

### Cambiar nombre del tópico

Línea ~108:
```c
"temperatura"  // Cambiar a "temp", "sensor/temperature", etc.
```

### Cambiar nombre del nodo

Línea ~101:
```c
"esp32_temp_sensor"  // Cambiar a tu nombre preferido
```

---

## ✅ Verificación del sistema

### Checklist completo:

- [ ] ESP32 conectado por USB
- [ ] Sensor DS18B20 conectado (GPIO 4, con pull-up 4.7kΩ)
- [ ] Código compilado y flasheado
- [ ] Monitor muestra "Esperando conexión..."
- [ ] micro-ROS Agent ejecutándose en PC
- [ ] Monitor ESP32 muestra "Conectado al micro-ROS Agent!"
- [ ] `ros2 topic list` muestra `/temperatura`
- [ ] `ros2 topic echo /temperatura` muestra datos

### Si algo falla:

**ESP32 no detecta sensor:**
- Verificar conexiones físicas
- Verificar resistencia pull-up 4.7kΩ
- Intentar otro GPIO (cambiar `ONE_WIRE_GPIO` en código)

**No se conecta al Agent:**
- Verificar que el Agent está corriendo: `./microros.sh agent-serial`
- Verificar puerto serial: `./microros.sh ports`
- Configurar permisos: `./microros.sh fix-permissions`
- Test conexión serial: `./microros.sh test-serial`
- Reiniciar ESP32 (botón RESET físico)

**No aparecen tópicos en ROS:**
- Verificar `ROS_DOMAIN_ID` (debe ser igual en ESP32 y PC)
- Verificar firewall
- Reiniciar Agent

---

## 🛠️ Comandos Útiles del Script

```bash
cd ~/Documentos/Github/sensores/microRostest/scripts

# Ver menú interactivo completo (19 opciones)
./microros.sh

# ESP32
./microros.sh build              # Compilar proyecto
./microros.sh flash              # Flashear ESP32
./microros.sh monitor            # Monitor serial
./microros.sh all                # Build + Flash + Monitor
./microros.sh clean              # Limpiar proyecto
./microros.sh menuconfig         # Configuración ESP-IDF

# Agent
./microros.sh agent-serial       # Iniciar Agent por serial
./microros.sh agent-udp          # Iniciar Agent por UDP

# ROS 2
./microros.sh topics             # Listar tópicos
./microros.sh listen             # Escuchar /temperatura
./microros.sh node-info          # Info del nodo
./microros.sh hz                 # Frecuencia de publicación

# Diagnóstico
./microros.sh ports              # Ver puertos USB
./microros.sh test-serial        # Test conexión serial
./microros.sh sysinfo            # Info del sistema
./microros.sh check-deps         # Verificar dependencias
```

---

## 🎓 Siguiente nivel

### 1. Añadir suscriptor (ESP32 recibe comandos)
```c
// Suscribirse a comandos del PC
rcl_subscription_t command_subscriber;
// ... implementar callback
```

### 2. Publicar múltiples tópicos
```c
// Temperatura + humedad + presión
rcl_publisher_t temp_pub;
rcl_publisher_t humidity_pub;
rcl_publisher_t pressure_pub;
```

### 3. Usar WiFi en lugar de Serial
```bash
idf.py menuconfig
# micro-ROS Settings → WiFi Configuration
```

Luego en PC:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 4. Integrar con nav2, MoveIt, etc.
Tu ESP32 ahora es un nodo ROS estándar, puede integrarse con cualquier sistema ROS 2.

---

## 📚 Recursos

- **Documentación del proyecto**: 
  - [README.md](../README.md) - Documentación completa
  - [INICIO_RAPIDO.md](INICIO_RAPIDO.md) - Guía rápida de inicio
  - [scripts/README.md](../scripts/README.md) - Documentación de scripts
- **Scripts útiles**: 
  - [microros.sh](../scripts/microros.sh) - Script unificado TODO-EN-UNO
  - [pc_temperature_subscriber.py](../scripts/pc_temperature_subscriber.py) - Nodo Python ejemplo
- **Documentación externa**:
  - [micro-ROS docs](https://micro.ros.org/) - Documentación oficial micro-ROS
  - [ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/) - Documentación ROS 2
  - [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/) - Documentación ESP-IDF

---

**¡Tu ESP32 ahora es un nodo ROS profesional! 🚀**
