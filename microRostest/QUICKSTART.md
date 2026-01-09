# 🚀 Inicio Rápido - micro-ROS con ESP32

## ⚡ Configuración en 5 minutos

### 1️⃣ Instalar micro-ROS Agent (solo una vez)

```bash
cd /home/lab-ros/Documentos/Github/microRostest
./scripts/install_microros_agent.sh
```

### 2️⃣ Usar el Helper Script (Recomendado)

```bash
./scripts/microros_helper.sh
```

El script te guiará por todas las opciones.

---

## 📝 Comandos Rápidos

### Para el ESP32:

```bash
# Entorno ESP-IDF
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
cd /home/lab-ros/Documentos/Github/microRostest

# Compilar
idf.py build

# Flashear
idf.py -p /dev/ttyUSB0 flash

# Monitor
idf.py monitor

# Todo junto
idf.py -p /dev/ttyUSB0 build flash monitor
```

### Para ROS 2:

```bash
# Terminal 1: Iniciar Agent
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2: Ver datos
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /temperatura
ros2 topic hz /temperatura
```

---

## 🔄 Flujo completo

```
Terminal 1 (ESP32):
┌─────────────────────────────────────────┐
│ cd microRostest                         │
│ source /home/lab-ros/esp/v5.5.2/       │
│        esp-idf/export.sh                │
│ idf.py build flash monitor              │
└─────────────────────────────────────────┘

Terminal 2 (Agent):
┌─────────────────────────────────────────┐
│ source /opt/ros/jazzy/setup.bash        │
│ ros2 run micro_ros_agent               │
│      micro_ros_agent serial             │
│      --dev /dev/ttyUSB0 -b 115200       │
└─────────────────────────────────────────┘

Terminal 3 (ROS 2):
┌─────────────────────────────────────────┐
│ source /opt/ros/jazzy/setup.bash        │
│ ros2 topic echo /temperatura            │
└─────────────────────────────────────────┘
```

---

## 🔧 Solución de Problemas Comunes

### "idf.py: no se encontró la orden"
```bash
source /home/lab-ros/esp/v5.5.2/esp-idf/export.sh
```

### "Permission denied" en /dev/ttyUSB0
```bash
sudo chmod 666 /dev/ttyUSB0
# O de forma permanente:
sudo usermod -a -G dialout $USER
# (requiere logout/login)
```

### No aparecen tópicos
1. Verificar que el Agent está corriendo
2. Verificar que el ESP32 está conectado
3. Reiniciar el ESP32

### CMakeLists.txt con errores en VSCode
- Click en "Python" (esquina inferior derecha)
- Cambiar a "CMake"

---

## 📁 Archivos importantes

- [README.md](README.md) - Documentación completa
- [sensor_temp.c](main/sensor_temp.c) - Código actual (solo sensor)
- [sensor_temp_microros.c.example](main/sensor_temp_microros.c.example) - Ejemplo con micro-ROS
- [microros_helper.sh](microros_helper.sh) - Script interactivo
- [install_microros_agent.sh](install_microros_agent.sh) - Instalador del agent

---

## 💡 Próximos pasos

1. **Activar micro-ROS en el código:**
   - Renombrar `sensor_temp.c` a `sensor_temp_backup.c`
   - Renombrar `sensor_temp_microros.c.example` a `sensor_temp.c`
   - Compilar y flashear

2. **Configurar WiFi (opcional):**
   ```bash
   idf.py menuconfig
   # micro-ROS Settings → WiFi Configuration
   ```

3. **Crear nodo ROS 2 en PC para procesar datos:**
   ```bash
   ros2 run rqt_plot rqt_plot /temperatura/data
   ```

---

## 📚 Ver documentación completa

Para más detalles, consulta [README.md](README.md)
