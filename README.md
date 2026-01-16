# 🌡️ Sistema de Monitoreo de Sensores con ROS 2

<div align="center">

**Última actualización:** 15 de enero de 2026

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![ESP-IDF 5.5.2](https://img.shields.io/badge/ESP--IDF-5.5.2-green.svg)](https://docs.espressif.com/projects/esp-idf/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-Serial-orange.svg)](https://micro.ros.org/)
[![MongoDB](https://img.shields.io/badge/MongoDB-Atlas-green.svg)](https://www.mongodb.com/cloud/atlas)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

**Sistema IoT de monitoreo ambiental usando ESP32 + ROS 2 + micro-ROS + MongoDB**

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
- ☁️ **MongoDB Atlas integrado** - Almacenamiento automático en nube
- 💾 **Respaldo local** - JSON Lines para persistencia local
- 🐍 **Python subscribers** - Nodos de ejemplo para procesar datos
- 📚 **Documentación completa** - Guías paso a paso y troubleshooting

---

## 📦 Proyectos

### 🌡️🧪 [microRostest](./microRostest/) - Sensor CWT-BL (pH + Temperatura)

Sistema completo de monitoreo ambiental con sensor CWT-BL dual (pH y temperatura) y ESP32.

**Stack tecnológico:**
- ESP32 + ESP-IDF 5.5.2
- Sensor CWT-BL (pH + Temperatura analógico)
- micro-ROS (Serial transport)
- ROS 2 Jazzy
- MongoDB Atlas (almacenamiento en nube)

**Características:**
- ✅ Lectura dual: pH (0-14) y Temperatura (-20°C a 80°C)
- ✅ Publicación en topics `/ph` y `/temperatura` (std_msgs/Float32)
- ✅ Frecuencia: 0.25 Hz (cada 4 segundos)
- ✅ Almacenamiento en MongoDB Atlas con Python
- ✅ Script unificado `microros.sh` con 19 opciones
- ✅ Subscribers Python con estadísticas en tiempo real
- ✅ Respaldo local en archivos JSON

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
| Sensor Dual | CWT-BL | pH (0-14) + Temp (-20 a 80°C), Analógico | $15-25 USD |
| Cable | USB-A a Micro-USB | Para programación y alimentación | $1-2 USD |

**Costo total:** ~$20-35 USD

### Diagrama de conexiones

```
┌──────────────────────────────────────────────────┐
│                    ESP32                         │
│                                                  │
│  5V ●──────────────────● VCC  Sensor CWT-BL     │
│                        │                         │
│  GPIO39 (ADC) ●────────● Temp Out (0-3.3V)      │
│                        │                         │
│  GPIO36 (ADC) ●────────● pH Out (0-3.3V)        │
│                        │                         │
│  GND ●─────────────────● GND                     │
│                                                  │
│  USB ◄─────────► PC (Ubuntu + MongoDB Atlas)    │
│        /dev/ttyUSB0                              │
└──────────────────────────────────────────────────┘
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

- **Frecuencia de publicación:** 0.25 Hz (cada 4s, configurable)
- **Precisión temperatura:** Calibrable (fórmula ajustable)
- **Rango temperatura:** -20°C a 80°C
- **Precisión pH:** ±0.1 pH (con calibración)
- **Rango pH:** 0 a 14
- **Consumo ESP32:** ~80mA @ 5V
- **Almacenamiento:** MongoDB Atlas (nube)

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

- [x] Sensor CWT-BL (pH + temperatura)
- [x] Integración con MongoDB Atlas
- [x] Almacenamiento automático en nube
- [ ] Dashboard web en tiempo real (Grafana)
- [ ] Múltiples ESP32 con IDs únicos
- [ ] Colección de dispositivos en MongoDB
- [ ] Implementar transporte WiFi (UDP)
- [ ] Alertas por valores fuera de rango
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
