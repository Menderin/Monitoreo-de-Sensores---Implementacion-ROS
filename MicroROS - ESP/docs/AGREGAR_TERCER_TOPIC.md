# Guía: Agregar un Tercer Topic a micro-ROS ESP32

## Contexto

Actualmente tienes **2 publishers** (temperatura y pH). Para agregar un tercer topic, necesitas modificar varios archivos y considerar las **limitaciones de memoria** de micro-ROS mencionadas en el [issue #526](https://github.com/micro-ROS/micro_ros_setup/issues/526).

---

## ⚠️ Problema Común (Issue #526)

El issue que compartiste menciona que **micro-ROS tiene límites de memoria** para crear múltiples publishers/subscribers en ESP32:

> [!WARNING]
> **Síntoma**: Solo se crean los primeros 2-3 publishers, los demás fallan con `RCL_RET_ERROR`
> 
> **Causa**: Falta de memoria en el heap de micro-ROS

### Solución: Aumentar la Memoria

Necesitas modificar la configuración de memoria en `sdkconfig` o usando menuconfig:

```bash
# Opción 1: Usar menuconfig
idf.py menuconfig

# Navega a:
# Component config → 
#   micro-ROS → 
#     Memory allocation → 
#       Heap size
```

**Valores recomendados para 3 publishers**:
- `CONFIG_MICRO_ROS_APP_HEAP_SIZE`: **20000** (mínimo 15000)
- Si usas timers/executors, puede necesitar más

---

## 📁 Archivos a Modificar

Para agregar un tercer topic (ejemplo: "turbidez"), modifica estos **8 archivos**:

### 1. ⚠️ `colcon.meta` - **CRÍTICO**

> [!CAUTION]
> **Este es el archivo MÁS IMPORTANTE**. Sin modificarlo, el tercer publisher NO se creará aunque todo lo demás esté correcto.

**Qué modificar**: Aumentar los límites de publishers en dos secciones

**Ubicación**: `components/micro_ros_espidf_component/colcon.meta`

```diff
"rmw_microxrcedds": {
    "cmake-args": [
        "-DRMW_UXRCE_XML_BUFFER_LENGTH=400",
        "-DRMW_UXRCE_TRANSPORT=udp",
        "-DRMW_UXRCE_MAX_NODES=1",
-       "-DRMW_UXRCE_MAX_PUBLISHERS=2",
+       "-DRMW_UXRCE_MAX_PUBLISHERS=3",
-       "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=2",
+       "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=3",
        "-DRMW_UXRCE_MAX_SERVICES=1",
        "-DRMW_UXRCE_MAX_CLIENTS=1",
        "-DRMW_UXRCE_MAX_HISTORY=1"
    ]
},
"embeddedrtps": {
    "cmake-args": [
-       "-DERTPS_MAX_PUBLISHERS=2",
+       "-DERTPS_MAX_PUBLISHERS=3",
-       "-DERTPS_MAX_SUBSCRIPTIONS=2",
+       "-DERTPS_MAX_SUBSCRIPTIONS=3",
        "-DERTPS_MAX_SERVICES=1",
        "-DERTPS_MAX_CLIENTS=1",
        "-DERTPS_MAX_HISTORY=10"
    ]
}
```

> [!IMPORTANT]
> Después de modificar `colcon.meta`, debes hacer un **clean build**:
> ```bash
> idf.py fullclean
> idf.py build
> ```

---

### 2. `config.h`

**Ubicación**: `main/versions/wifi/include/config.h`

**Qué agregar**: 
- Nombre del nuevo topic
- Canal ADC si usa un nuevo sensor
- Constantes de calibración

```diff
// Canales ADC
#define ADC_TEMP_CHANNEL    ADC_CHANNEL_3  // GPIO39 (VN) - Temperatura
#define ADC_PH_CHANNEL      ADC_CHANNEL_0  // GPIO36 (VP) - pH
+#define ADC_TURBIDITY_CHANNEL ADC_CHANNEL_6  // GPIO34 - Turbidez

// Calibración
#define TEMP_OFFSET_CAL     -1.5
#define PH_SLOPE            0.00375
#define PH_INTERCEPT        0.58
+// Turbidez: conversión de voltaje a NTU (Nephelometric Turbidity Units)
+#define TURBIDITY_SLOPE     0.5
+#define TURBIDITY_OFFSET    0.0

// Configuración de Nodo ROS
#define NODE_NAME               "esp32_sensor_node"
#define TOPIC_TEMPERATURE       "temperatura"
#define TOPIC_PH                "ph"
+#define TOPIC_TURBIDITY         "turbidez"
```

---

### 3. `ros_publisher.h`

**Ubicación**: `main/versions/wifi/include/ros_publisher.h`

**Qué agregar**: Campo en la estructura de datos

```diff
typedef struct {
    float temperature;
    float ph;
+   float turbidity;
} sensor_data_t;
```

---

### 4. `ros_publisher.c`

**Ubicación**: `main/versions/wifi/src/ros_publisher.c`

**Cambios necesarios**:
1. Declarar nueva variable de publisher
2. Declarar nuevo mensaje
3. Inicializar el publisher
4. Publicar el dato

```diff
// Variables privadas (línea ~45)
static rcl_publisher_t temperature_publisher;
static rcl_publisher_t ph_publisher;
+static rcl_publisher_t turbidity_publisher;

static std_msgs__msg__Float32 temperature_msg;
static std_msgs__msg__Float32 ph_msg;
+static std_msgs__msg__Float32 turbidity_msg;

// En ros_publisher_init() - después de crear publisher de pH (línea ~114)
RCCHECK(rclc_publisher_init_default(
    &ph_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    TOPIC_PH));
ESP_LOGI(TAG, "Publicador creado: '/%s'", TOPIC_PH);
+
+// Crear publicador de turbidez
+RCCHECK(rclc_publisher_init_default(
+    &turbidity_publisher,
+    &node,
+    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
+    TOPIC_TURBIDITY));
+ESP_LOGI(TAG, "Publicador creado: '/%s'", TOPIC_TURBIDITY);

// En ros_publisher_publish() - actualizar mensaje (línea ~137)
temperature_msg.data = data->temperature;
ph_msg.data = data->ph;
+turbidity_msg.data = data->turbidity;

// Publicar (línea ~141)
RCSOFTCHECK(rcl_publish(&temperature_publisher, &temperature_msg, NULL));
RCSOFTCHECK(rcl_publish(&ph_publisher, &ph_msg, NULL));
+RCSOFTCHECK(rcl_publish(&turbidity_publisher, &turbidity_msg, NULL));

-ESP_LOGI(TAG, "Temp: %.2f °C | pH: %.2f", data->temperature, data->ph);
+ESP_LOGI(TAG, "Temp: %.2f °C | pH: %.2f | Turbidez: %.2f NTU", 
+         data->temperature, data->ph, data->turbidity);

// En ros_publisher_deinit() - limpiar (línea ~156)
rcl_publisher_fini(&temperature_publisher, &node);
rcl_publisher_fini(&ph_publisher, &node);
+rcl_publisher_fini(&turbidity_publisher, &node);
```

---

### 5. `sensor_manager.h`

**Ubicación**: `main/versions/wifi/include/sensor_manager.h`

**Qué agregar**: Función para leer el nuevo sensor

Agregar al final del archivo (antes del `#endif`):

```c
/**
 * @brief Lee el valor de turbidez
 * @return Valor de turbidez en NTU
 */
float sensor_read_turbidity(void);
```

---

### 6. `sensor_manager.c`

**Ubicación**: `main/versions/wifi/src/sensor_manager.c`

**Cambios necesarios**:
1. Implementar la función de lectura del sensor
2. Si usa un nuevo canal ADC, agregarlo en la inicialización

```c
// Agregar al final del archivo
float sensor_read_turbidity(void)
{
    int adc_raw;
    
    // Leer ADC
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_TURBIDITY_CHANNEL, &adc_raw));
    
    // Convertir a voltaje (mV)
    int voltage = adc_raw * 3300 / 4095;
    
    // Convertir a NTU usando calibración
    float turbidity = (voltage * TURBIDITY_SLOPE) + TURBIDITY_OFFSET;
    
    return turbidity;
}
```

> [!NOTE]
> Si tu sensor de turbidez usa el mismo bus ADC, no necesitas modificar `sensor_manager_init()` ya que el handle ADC ya está configurado.

---

### 7. `main.c`

**Ubicación**: `main/versions/wifi/src/main.c`

**Qué modificar**: Leer y enviar el nuevo dato

```diff
// En micro_ros_task() - dentro del while loop (línea ~51)
// Leer sensores
data.temperature = sensor_read_temperature();
data.ph = sensor_read_ph();
+data.turbidity = sensor_read_turbidity();

// En app_main() - prueba de sensores (línea ~95)
ESP_LOGI(TAG, "   Temperatura: %.2f °C", sensor_read_temperature());
ESP_LOGI(TAG, "   pH: %.2f", sensor_read_ph());
+ESP_LOGI(TAG, "   Turbidez: %.2f NTU", sensor_read_turbidity());
```

---

### 8. `sdkconfig`

**Opción 1: Usando menuconfig (Recomendado)**

```bash
cd microRostest
idf.py menuconfig
```

Navega y modifica:
```
Component config →
  micro-ROS settings →
    Memory allocation →
      [*] Use custom memory pool
      (20000) Custom heap size  # Aumentar de 15000 a 20000
```

**Opción 2: Modificar sdkconfig directamente**

Busca y modifica:

```bash
CONFIG_MICRO_ROS_APP_HEAP_SIZE=20000
```

> [!IMPORTANT]
> Sin este cambio, probablemente verás el error `RCL_RET_ERROR` al crear el tercer publisher.

---

## 📝 Resumen de Archivos Modificados

| Archivo | Línea Aprox | Qué Agregar |
|---------|-------------|-------------|
| **`colcon.meta`** ⚠️ | **40-41, 49-50** | **Límites de MAX_PUBLISHERS y MAX_SUBSCRIPTIONS** |
| `config.h` | 17, 30, 52 | Canal ADC, constantes, nombre del topic |
| `ros_publisher.h` | 18 | Campo `turbidity` en struct |
| `ros_publisher.c` | 46, 49, 114, 138, 142, 157 | Publisher, mensaje, init, publish, cleanup |
| `sensor_manager.h` | Final | Declaración de función |
| `sensor_manager.c` | Final | Implementación de lectura |
| `main.c` | 52, 96 | Leer sensor y mostrar valor |
| `sdkconfig` | - | Aumentar heap size |

> [!CAUTION]
> El orden de importancia es:
> 1. **`colcon.meta`** - Sin esto, nada funciona
> 2. **`sdkconfig`** - Sin memoria suficiente, falla
> 3. Los demás archivos - Implementación del código

---

## ✅ Verificación

Después de los cambios, ejecuta:

```bash
cd microRostest
idf.py fullclean  # Importante después de modificar colcon.meta
idf.py build
idf.py flash monitor
```

Deberías ver en los logs:
```
Publicador creado: '/temperatura'
Publicador creado: '/ph'
Publicador creado: '/turbidez'  ✅
```

Y para verificar en ROS 2:
```bash
ros2 topic list
# Deberías ver:
# /temperatura
# /ph
# /turbidez

ros2 topic echo /turbidez
```

---

## 🚨 Troubleshooting

### Error: `RCL_RET_ERROR` al crear el tercer publisher

**Solución**: 
1. Verifica que modificaste `colcon.meta` 
2. Aumenta `CONFIG_MICRO_ROS_APP_HEAP_SIZE` en sdkconfig
3. Haz `idf.py fullclean` y vuelve a compilar

### Error: Compilación falla con symbol not found

**Solución**: Verifica que agregaste todas las declaraciones en los `.h` correspondientes

### El topic se crea pero no publica datos

**Solución**: Verifica que estás leyendo el sensor en `main.c` y pasando el dato correctamente

---

## 💡 Mejora Futura: Usar Executors

Si planeas agregar **más de 3 publishers**, considera usar **executors con timers** como en el issue #526, que es más eficiente en memoria que crear múltiples publishers individuales.
