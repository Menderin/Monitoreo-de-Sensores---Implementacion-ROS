# 🛠️ Scripts y Herramientas

Esta carpeta contiene scripts de utilidad para facilitar el desarrollo con micro-ROS y ESP32.

## 📄 Archivos

### 🔨 build_and_flash.sh
**Compilar y flashear el ESP32**
```bash
./build_and_flash.sh
```
- Detecta automáticamente el puerto USB
- Menú interactivo con opciones:
  1. Solo compilar
  2. Compilar y flashear
  3. Compilar, flashear y monitorear
  4. Solo monitorear
  5. Limpiar y compilar

**Uso típico:** Opción 3 (build + flash + monitor)

---

### 🎛️ microros_helper.sh
**Menú completo de herramientas**
```bash
./microros_helper.sh
```
Opciones disponibles:
- **ESP32:** Build, flash, monitor, limpiar, menuconfig
- **ROS 2:** Iniciar agent (serial/UDP), ver tópicos, monitorear temperatura
- **Utilidades:** Ver puertos, permisos USB, diagnosticar

**Recomendado para:** Flujo de trabajo completo

---

### 📦 install_microros_agent.sh
**Instalar micro-ROS Agent en el PC** (solo una vez)
```bash
./install_microros_agent.sh
```
Instala el `ros2 run micro_ros_agent` necesario para que el PC se comunique con el ESP32.

**Cuándo usar:** Primera vez configurando el sistema, o si falta el agent

---

### 🐍 pc_temperature_subscriber.py
**Nodo ROS 2 en Python - Suscriptor de temperatura**
```bash
python3 pc_temperature_subscriber.py
```
Ejemplo de nodo que:
- Se suscribe al tópico `/temperatura`
- Convierte °C a °F
- Calcula estadísticas (min, max, promedio)
- Muestra alertas de temperatura

**Uso:** Como ejemplo o base para tus propios nodos

---

## 🚀 Flujo de trabajo típico

### Primera vez:
```bash
# 1. Instalar agent
./install_microros_agent.sh

# 2. Compilar y flashear ESP32
./build_and_flash.sh
# Seleccionar opción 3

# 3. En otra terminal: Iniciar agent
./microros_helper.sh
# Seleccionar opción 6

# 4. En otra terminal: Ver datos
python3 pc_temperature_subscriber.py
```

### Desarrollo día a día:
```bash
# Opción rápida todo-en-uno
./microros_helper.sh
```

---

## 📝 Notas

- Todos los scripts deben ejecutarse desde el directorio raíz del proyecto
- Los scripts detectan automáticamente el puerto USB del ESP32
- Si hay errores de permisos: `sudo chmod 666 /dev/ttyUSB0`

---

## 🔗 Enlaces útiles

- [README principal](../README.md) - Documentación completa
- [Inicio rápido](../QUICKSTART.md) - Guía de 5 minutos
- [Uso como nodo ROS](../USO_NODO_ROS.md) - Detalles del nodo ROS
