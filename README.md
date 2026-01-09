# 🌡️ Sistema de Monitoreo de Sensores con ROS 2

<div align="center">

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![ESP-IDF 5.5.2](https://img.shields.io/badge/ESP--IDF-5.5.2-green.svg)](https://docs.espressif.com/projects/esp-idf/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-Serial-orange.svg)](https://micro.ros.org/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

**Sistema IoT de monitoreo ambiental usando ESP32 + ROS 2 + micro-ROS**

[Características](#-características) •
[Proyectos](#-proyectos) •
[Inicio Rápido](#-inicio-rápido) •
[Documentación](#-documentación) •
[Hardware](#-hardware)

</div>

---

## 📖 Descripción

Repositorio de proyectos de **sensores IoT integrados con ROS 2** mediante **micro-ROS**, permitiendo la comunicación entre microcontroladores ESP32 y el ecosistema robótico ROS. Ideal para aplicaciones de monitoreo ambiental, prototipado rápido y aprendizaje de sistemas embebidos con ROS.

### ✨ Características

- 🤖 **Integración ROS 2 nativa** - Nodos micro-ROS ejecutándose en ESP32
- 📡 **Comunicación UART Serial** - Conexión directa ESP32 ↔ PC (115200 baud)
- 🔧 **Scripts de automatización** - Herramientas unificadas para build, flash y monitoreo
- 📊 **Datos en tiempo real** - Publicación continua a topics ROS 2
- 🐍 **Python subscribers** - Nodos de ejemplo para procesar datos
- 📚 **Documentación completa** - Guías paso a paso y troubleshooting

---

## 📦 Proyectos

### 🌡️ [microRostest](./microRostest/) - Sensor de Temperatura DS18B20

Sistema completo de monitoreo de temperatura con sensor DS18B20 y ESP32.

**Stack tecnológico:**
- ESP32 + ESP-IDF 5.5.2
- Sensor DS18B20 (OneWire)
- micro-ROS (Serial transport)
- ROS 2 Jazzy

**Características:**
- ✅ Lectura digital de temperatura (-55°C a +125°C)
- ✅ Publicación en topic `/temperatura` (std_msgs/Float32)
- ✅ Frecuencia: 0.5 Hz (cada 2 segundos)
- ✅ Script unificado `microros.sh` con 19 opciones
- ✅ Subscribers Python con estadísticas y alertas

**[Ver documentación completa →](./microRostest/README.md)**

---

## 🚀 Inicio Rápido

### Prerrequisitos

```bash
# Sistema operativo
Ubuntu 22.04 LTS (o compatible)

# Software instalado
- ROS 2 Jazzy
- ESP-IDF v5.5.2
- micro-ROS workspace
- Python 3.12+
```

### Instalación en 3 pasos

```bash
# 1. Clonar repositorio
git clone https://github.com/Menderin/sensores.git
cd sensores/microRostest

# 2. Flashear ESP32 (primera vez)
cd scripts
./microros.sh  # Opción 4: Build + Flash + Monitor

# 3. Iniciar Agent y verificar
./microros.sh  # Opción 8: Iniciar Agent
./microros.sh  # Opción 11: Escuchar /temperatura (nueva terminal)
```

### Verificación rápida

```bash
# Terminal 1: Agent
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2: Ver datos
source /opt/ros/jazzy/setup.bash
ros2 topic echo /temperatura
```

Deberías ver:
```
data: 25.31
---
data: 25.25
---
```

✅ **¡Sistema funcionando!**

---

## 📚 Documentación

### Por Proyecto

| Proyecto | README | Inicio Rápido | Estructura | Uso Avanzado |
|----------|--------|---------------|------------|--------------|
| **microRostest** | [📄](./microRostest/README.md) | [🚀](./microRostest/docs/INICIO_RAPIDO.md) | [🏗️](./microRostest/docs/ESTRUCTURA_PROYECTO.md) | [⚙️](./microRostest/docs/USO_NODO_ROS.md) |

### Guías Generales

- 📘 **[Script unificado `microros.sh`](./microRostest/scripts/README.md)** - Documentación completa del script de automatización
- 🔧 **[Troubleshooting](./microRostest/README.md#-troubleshooting)** - Solución de problemas comunes
- 🎓 **[Ruta de aprendizaje](./microRostest/docs/INICIO_RAPIDO.md)** - Orden recomendado de lectura

---

## 🔌 Hardware

### Componentes utilizados

| Componente | Modelo | Especificación | Precio aprox. |
|------------|--------|----------------|---------------|
| Microcontrolador | ESP32-DevKit | ESP32-D0WDQ6, Dual-Core @ 240MHz | $4-8 USD |
| Sensor Temp. | DS18B20 | -55°C a +125°C, OneWire, ±0.5°C | $2-4 USD |
| Resistencia | Pull-up | 4.7kΩ, 1/4W | < $0.10 USD |
| Cable | USB-A a Micro-USB | Para programación y alimentación | $1-2 USD |

**Costo total:** ~$7-15 USD

### Diagrama de conexiones

```
┌─────────────────────────────────────────────┐
│                  ESP32                      │
│                                             │
│  3.3V ●────┬────────────● VCC  DS18B20     │
│            │            │                   │
│            └──[4.7kΩ]──┤                   │
│                         │                   │
│  GPIO4 ●───────────────● DATA              │
│                         │                   │
│  GND ●─────────────────● GND               │
│                                             │
│  USB ◄────────► PC (Ubuntu)                │
│       /dev/ttyUSB0                          │
└─────────────────────────────────────────────┘
```

---

## 🛠️ Tecnologías

<table>
<tr>
<td align="center" width="140">
<img src="https://raw.githubusercontent.com/ros-infrastructure/artwork/master/ros_logo.svg" width="100" alt="ROS 2"/><br>
<b>ROS 2 Jazzy</b>
</td>
<td align="center" width="140">
<img src="https://docs.espressif.com/projects/esp-idf/en/stable/esp32/_static/espressif-logo.svg" width="100" alt="ESP-IDF"/><br>
<b>ESP-IDF 5.5.2</b>
</td>
<td align="center" width="140">
<img src="https://micro.ros.org/img/micro-ROS_big_logo.png" width="100" alt="micro-ROS"/><br>
<b>micro-ROS</b>
</td>
<td align="center" width="140">
<img src="https://upload.wikimedia.org/wikipedia/commons/c/c3/Python-logo-notext.svg" width="100" alt="Python"/><br>
<b>Python 3.12</b>
</td>
</tr>
</table>

---

## 📊 Características del Sistema

### Comunicación

- **Protocolo:** DDS-XRCE (micro-ROS)
- **Transporte:** Serial UART
- **Velocidad:** 115200 baudios
- **Latencia:** < 100ms
- **QoS:** Best Effort

### Performance

- **Frecuencia de publicación:** 0.5 Hz (configurable)
- **Precisión del sensor:** ±0.5°C
- **Rango de temperatura:** -55°C a +125°C
- **Consumo ESP32:** ~80mA @ 3.3V

---

## 🤝 Contribuir

¡Las contribuciones son bienvenidas! Si quieres agregar un nuevo proyecto de sensor o mejorar los existentes:

1. Fork el repositorio
2. Crea tu rama de feature (`git checkout -b feature/nuevo-sensor`)
3. Commit tus cambios (`git commit -m 'Add: Sensor XYZ con micro-ROS'`)
4. Push a la rama (`git push origin feature/nuevo-sensor`)
5. Abre un Pull Request

---

## 📝 To-Do / Roadmap

- [ ] Agregar sensor DHT22 (temperatura + humedad)
- [ ] Implementar transporte WiFi (UDP)
- [ ] Dashboard web en tiempo real
- [ ] Integración con InfluxDB + Grafana
- [ ] Multi-sensor (varios sensores en un ESP32)
- [ ] OTA updates para firmware

---

## 📄 Licencia

Este proyecto está bajo la Licencia MIT - ver el archivo [LICENSE](LICENSE) para detalles.

---

## 👤 Autor

**Menderin**

- GitHub: [@Menderin](https://github.com/Menderin)
- Proyecto: [sensores](https://github.com/Menderin/sensores)

---

## 🙏 Agradecimientos

- [micro-ROS](https://micro.ros.org/) - Framework de ROS 2 para microcontroladores
- [ESP-IDF](https://github.com/espressif/esp-idf) - Framework de desarrollo de Espressif
- [ROS 2](https://docs.ros.org/) - Robot Operating System
- Comunidad open-source de robótica e IoT

---

<div align="center">

**⭐ Si te gusta este proyecto, dale una estrella en GitHub ⭐**

[Reportar Bug](https://github.com/Menderin/sensores/issues) •
[Solicitar Feature](https://github.com/Menderin/sensores/issues) •
[Documentación](./microRostest/docs/)

</div>
