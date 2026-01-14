# 🚀 Inicio Rápido - micro-ROS + ESP32 + DS18B20

**Objetivo:** Poner en funcionamiento el sistema de temperatura en menos de 5 minutos.

**Requisitos previos:**
- ✅ ESP32 con firmware ya flasheado y conectado por USB
- ✅ micro-ROS Agent instalado en `~/microros_ws`
- ✅ ROS 2 Jazzy instalado en `/opt/ros/jazzy`
- ✅ Permisos USB configurados (grupo `dialout`)

> 💡 **Primera vez?** Ver [README.md](../README.md) para instalación completa.

---

## ⚡ Inicio Rápido (4 pasos)

### 📍 Paso 1: Verificar ESP32

Conecta el ESP32 por USB y verifica que está esperando el Agent:

```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
./microros.sh monitor
```

**Debe mostrar:**
```
I (2981) MICRO_ROS_TEMP: 🌡️ Lectura inicial: 25.00 °C
I (2981) MICRO_ROS_TEMP: 🔍 Esperando conexión con micro-ROS Agent...
W (13001) MICRO_ROS_TEMP: ⏳ Esperando agente... intento 1/10
```

**Salir:** `Ctrl + ]`

---

### 🔌 Paso 2: Iniciar micro-ROS Agent

El Agent es el **puente de comunicación** entre ESP32 y ROS 2. **DEBE estar corriendo** para que el sistema funcione.

**Opción A: Menú interactivo (recomendado para principiantes)**
```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
./microros.sh
# Selecciona: Opción 8 - Iniciar Agent (Serial)
```

**Opción B: Comando directo**
```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
./microros.sh agent-serial
```

**Opción C: Comando manual**
```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Salida esperada (conexión exitosa):**
```
[INFO] [TermiosAgentLinux]: Serial port opened at /dev/ttyUSB0
[INFO] [Root]: create_client | client_key: 0x12345678, session_id: 0x81
```

> ⚠️ **El Agent debe permanecer ejecutándose.** No cierres esta terminal.

---

### ✅ Paso 3: Verificar Conexión

En una **nueva terminal**, verifica que el tópico está disponible:

**Opción A: Comando directo**
```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
./microros.sh topics
```

**Opción B: Menú interactivo**
```bash
./microros.sh
# Selecciona: Opción 10 - Listar tópicos
```

**Debe incluir:**
```
/parameter_events
/rosout
/temperatura      ← Este es nuestro tópico
```

---

### 📊 Paso 4: Ver Datos de Temperatura

**Opción A: Menú interactivo**
```bash
./microros.sh
# Selecciona: Opción 11 - Escuchar /temperatura
```

**Opción B: Comando directo**
```bash
./microros.sh listen
```

**Opción C: Nodo Python con estadísticas**
```bash
python3 pc_temperature_subscriber.py
```
**Salida:**
```
🌡️ [15:30:45] Temp: 25.12°C (77.22°F) | Estado: Normal | #Lectura: 1
🌡️ [15:30:47] Temp: 25.18°C (77.32°F) | Estado: Normal | #Lectura: 2
📊 Estadísticas (últimas 10 lecturas): Promedio=25.15°C | Min=25.06°C | Max=25.23°C
```

---

## 🔄 Flujo de Trabajo Típico (3 Terminales)

```bash
# Terminal 1: Agent (DEBE estar corriendo)
cd ~/Documentos/Github/sensores/microRostest/scripts
./microros.sh agent-serial

# Terminal 2: Monitor ESP32 (opcional - para ver logs)
./microros.sh monitor

# Terminal 3: Ver datos
./microros.sh listen
# O
python3 pc_temperature_subscriber.py
```

---

## ⚙️ ¿Qué es el micro-ROS Agent?

El **micro-ROS Agent** es un componente **CRÍTICO** que actúa como puente de comunicación:

```
ESP32 (micro-ROS) ←→ Agent (PC) ←→ ROS 2 (PC)
```

**Funciones del Agent:**
- 🔌 Recibe datos del ESP32 por puerto serial (`/dev/ttyUSB0` @ 115200 baud)
- 🔄 Convierte mensajes micro-ROS (DDS-XRCE) → ROS 2 (DDS)
- 📡 Publica datos en tópicos ROS 2 (como `/temperatura`)
- 🎯 Permite que nodos ROS 2 se comuniquen con el ESP32

**Sin el Agent corriendo:**
- ❌ El ESP32 queda esperando conexión (no crashea, solo espera)
- ❌ No aparece el tópico `/temperatura` en ROS 2
- ❌ Los datos del sensor no llegan al PC

**Tipos de transporte del Agent:**

```bash
# Serial/UART (configuración actual del proyecto)
./microros.sh agent-serial
# Equivalente a:
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# UDP/WiFi (si el ESP32 tuviera WiFi configurado)
./microros.sh agent-udp
# Equivalente a:
# ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

---

## 🛠️ Formas de Usar el Script

### Menú Interactivo (Recomendado)

```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
./microros.sh
```

Se mostrará un **menú con 19 opciones** organizadas en categorías:
- 📟 **ESP32 Development** (7 opciones): Build, flash, monitor, etc.
- 🌐 **micro-ROS Agent** (2 opciones): Iniciar Agent serial/UDP
- 📊 **ROS 2 Monitor** (4 opciones): Ver tópicos, nodos, datos
- ⚙️ **Installation** (3 opciones): Instalar Agent, dependencias, permisos
- 🔍 **Diagnostics** (3 opciones): Verificar sistema, puertos, logs

### Comandos Directos (Para usuarios avanzados)

```bash
cd ~/Documentos/Github/sensores/microRostest/scripts

# Comandos directos sin menú
./microros.sh agent-serial    # ★ Iniciar Agent (el más importante)
./microros.sh monitor         # Ver logs del ESP32
./microros.sh topics          # Listar tópicos ROS 2
./microros.sh listen          # Escuchar /temperatura
./microros.sh node-info       # Info del nodo ESP32
./microros.sh hz              # Frecuencia de publicación
./microros.sh ports           # Ver puertos USB disponibles
./microros.sh check-deps      # Verificar dependencias
```

---

## ❗ Problemas Comunes

### El Agent no inicia (error: comando no encontrado)

**Causa:** micro-ROS Agent no instalado

**Solución:**
```bash
./microros.sh install-agent
```

### Error: "Permission denied /dev/ttyUSB0"

**Causa:** Usuario sin permisos USB

**Solución:**
```bash
./microros.sh fix-permissions
# Luego cerrar sesión y volver a entrar
```

### El tópico `/temperatura` no aparece

**Checklist:**
1. ✅ El Agent está corriendo? (debe mostrar logs)
2. ✅ El ESP32 está conectado y encendido?
3. ✅ El monitor muestra "Esperando agente..."?

**Solución:** Reiniciar ESP32 (botón RESET físico) con el Agent corriendo

### Puerto ocupado (Agent no puede abrirlo)

**Causa:** Otro proceso usa el puerto (monitor, otro Agent)

**Solución:**
```bash
./microros.sh ports          # Ver qué está usando el puerto
# Cerrar monitors/agents anteriores o ejecutar:
sudo fuser -k /dev/ttyUSB0   # Matar procesos que usan el puerto
```

---

## 📝 Datos Técnicos

| Parámetro | Valor |
|-----------|-------|
| **Puerto serial** | `/dev/ttyUSB0` |
| **Baudrate** | 115200 |
| **Tópico ROS 2** | `/temperatura` |
| **Tipo de mensaje** | `std_msgs/msg/Float32` |
| **Frecuencia** | 0.5 Hz (cada 2 segundos) |
| **Transporte** | Serial UART (NO WiFi) |
| **Nodo ESP32** | `/micro_ros_esp32_node` |

---

## 🎯 Verificación de Sistema Funcionando

**Indicadores de éxito:**

1. ✅ Monitor ESP32 muestra: `✅ Conectado al micro-ROS Agent!`
2. ✅ Agent muestra: `create_client | client_key: 0x...`
3. ✅ `./microros.sh topics` lista `/temperatura`
4. ✅ `./microros.sh listen` muestra datos actualizándose

---

## 📚 Más Información

- **Documentación completa:** [README.md](../README.md)
- **Scripts detallados:** [scripts/README.md](../scripts/README.md)
- **Hardware y conexiones:** Ver README.md sección "Hardware"
- **Compilar firmware:** Ver README.md sección "Instalación"

---

**¡Sistema listo! 🎉**
