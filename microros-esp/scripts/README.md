# 🛠️ Scripts y Herramientas — ESP32 micro-ROS

Carpeta de utilidades para desarrollo con micro-ROS + ESP32 + ROS 2 sobre **WiFi/UDP**.

**Última actualización:** 19 de marzo de 2026

---

## 📄 microros.sh — Script principal

Interfaz interactiva para compilar, flashear, diagnosticar y operar el firmware ESP32 con micro-ROS.

### Uso

```bash
cd microros-esp/scripts
./microros.sh
```

### Menú interactivo (15 opciones)

```
  ESP32 — Build & Flash
    1)  Monitor serial
    2)  Build + Flash + Monitor
    3)  Limpiar proyecto (fullclean)
    4)  Configuración (menuconfig)

  micro-ROS Agent
    5)  Agent (Serial/UART)
    6)  Agent (UDP/WiFi)         ← modo principal del proyecto

  ROS 2 — Monitoreo
    7)  Ver tópicos
    8)  Frecuencia de publicación (hz)

  Instalación
    9)  Instalar micro-ROS Agent
    10) Verificar dependencias

  Diagnóstico
    11) Ver puertos seriales

  WiFi / Configuración
    12) Editar credenciales (.env)
    13) Mostrar IP del PC
    14) Generar wifi_config.h
    15) Verificar configuración WiFi
```

> **Modo de operación normal:** El proyecto usa **UDP/WiFi** (opción 6). La opción 5 (Serial/UART) está disponible solo para debugging directo.

---

## 📄 utils/calibracion_ph.py — Herramienta de calibración pH

Script interactivo para calibrar el sensor pH con regresión lineal.

### Uso

```bash
source /opt/ros/jazzy/setup.bash
cd microros-esp/scripts/utils
python3 calibracion_ph.py
```

### Flujo

1. **Selecciona el ESP32** a calibrar (menú con filtro por MAC — requiere micro-ROS Agent activo)
2. **Sumerge el sensor** en la primera solución buffer
3. **Presiona ESPACIO** → captura 10 muestras warmup + 30 reales → calcula mediana
4. **Ingresa el pH real** de esa solución
5. Repite para mínimo 2 buffers (ideal 3: pH 4, 6.86, 9.18)
6. **Presiona Enter** → calcula regresión lineal → muestra bloque listo para `config.h`

### Filtro por MAC

Al iniciar, el script:
- Consulta MongoDB para listar dispositivos registrados
- Escanea `/sensor_data` por 5 segundos detectando MACs activas
- Auto-registra MACs nuevas en la base de datos
- Permite seleccionar qué ESP32 calibrar

### Dependencias

```bash
pip install pymongo python-dotenv numpy
```

Requiere `database/.env` con `MONGO_URI` configurado.

---

## 🔧 Workflows Comunes

### Primera vez — compilar y flashear

```bash
cd microros-esp/scripts
./microros.sh
# → Opción 10: Verificar dependencias
# → Opción 12: Editar credenciales WiFi (.env)
# → Opción 2:  Build + Flash + Monitor
```

### Operación diaria — iniciar Agent UDP

```bash
# El Agent se gestiona como servicio systemd (desde menu.sh → 3 → a)
# Para lanzarlo manualmente en modo debug:
cd microros-esp/scripts
./microros.sh
# → Opción 6: Agent (UDP/WiFi)
```

### Ver datos en tiempo real

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic echo /sensor_data
```

Formato del mensaje `Float32MultiArray`:
```
data: [temperatura, pH, voltage_raw_ph, mac_part1, mac_part2]
```

---

## 💡 Tips

| Problema | Solución |
|---|---|
| ESP32 no flashea | `./microros.sh` → 11 (ver puertos), verificar permisos dialout |
| Agent no encuentra ESP32 | `./microros.sh` → 13 (verificar IP), actualizar `AGENT_IP` en `.env` |
| Credenciales WiFi cambiadas | `./microros.sh` → 12, luego → 2 (recompilar) |
| `ros2 topic list` vacío | Verificar `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |

---

**Ver documentación principal:** [README.md](../../README.md)
