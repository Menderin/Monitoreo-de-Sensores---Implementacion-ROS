# 🎯 Guía Rápida: ESP32 como Nodo ROS

## ✅ El código ya está listo

Tu archivo [sensor_temp.c](main/sensor_temp.c) ahora es un **nodo ROS completo** que:

✓ Lee temperatura del sensor DS18B20  
✓ Se conecta automáticamente al micro-ROS Agent en el PC  
✓ Publica datos en el tópico `/temperatura` cada 2 segundos  
✓ Funciona como un nodo ROS estándar  

---

## 🚀 Cómo usarlo (3 terminales)

### 📟 Terminal 1: Compilar y flashear ESP32

```bash
cd /home/lab-ros/Documentos/Github/microRostest
./build_and_flash.sh
```

O manualmente:
```bash
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

```bash
source /opt/ros/jazzy/setup.bash
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

```bash
# Configurar ROS
source /opt/ros/jazzy/setup.bash

# Ver todos los nodos
ros2 node list
# Salida: /esp32_temp_sensor

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

### Opción 3: Crear nodo suscriptor personalizado

```python
# temp_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TempSubscriber(Node):
    def __init__(self):
        super().__init__('temp_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'temperatura',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        temp_c = msg.data
        temp_f = (temp_c * 9/5) + 32
        self.get_logger().info(f'🌡️ {temp_c:.2f}°C = {temp_f:.2f}°F')

def main():
    rclpy.init()
    subscriber = TempSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Ejecutar:
```bash
python3 temp_subscriber.py
```

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
- Verificar que el Agent está corriendo
- Verificar puerto serial correcto (`/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.)
- Dar permisos: `sudo chmod 666 /dev/ttyUSB0`
- Reiniciar ESP32 (botón RESET)

**No aparecen tópicos en ROS:**
- Verificar `ROS_DOMAIN_ID` (debe ser igual en ESP32 y PC)
- Verificar firewall
- Reiniciar Agent

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

- **Este proyecto**: Ver [README.md](README.md)
- **Scripts útiles**: 
  - [build_and_flash.sh](build_and_flash.sh) - Compilar/flashear
  - [microros_helper.sh](microros_helper.sh) - Menú interactivo
- **micro-ROS docs**: https://micro.ros.org/
- **ROS 2 docs**: https://docs.ros.org/en/jazzy/

---

**¡Tu ESP32 ahora es un nodo ROS profesional! 🚀**
