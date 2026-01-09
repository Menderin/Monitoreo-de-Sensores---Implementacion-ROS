#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Librerías del sensor DS18B20
#include "onewire_bus.h"
#include "ds18b20.h"

// Librerías micro-ROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

// Configuración
#define ONE_WIRE_GPIO 4
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGW(TAG, "Failed status on line %d: %d. Continuing.",__LINE__,(int)temp_rc);}}

// Variables globales micro-ROS
rcl_publisher_t temperature_publisher;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
std_msgs__msg__Float32 temp_msg;

// Variables del sensor
ds18b20_device_handle_t ds18b20s = NULL;

static const char *TAG = "MICRO_ROS_TEMP";

// Callback del timer - Se ejecuta periódicamente para publicar la temperatura
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer != NULL && ds18b20s != NULL) {
        // Iniciar conversión de temperatura
        ds18b20_trigger_temperature_conversion(ds18b20s);
        vTaskDelay(pdMS_TO_TICKS(800)); // Esperar conversión (750-800ms)
        
        float temperature;
        if (ds18b20_get_temperature(ds18b20s, &temperature) == ESP_OK) {
            // Publicar temperatura en el tópico ROS
            temp_msg.data = temperature;
            RCSOFTCHECK(rcl_publish(&temperature_publisher, &temp_msg, NULL));
            ESP_LOGI(TAG, "📡 Publicado en ROS: %.2f °C", temperature);
        } else {
            ESP_LOGW(TAG, "⚠️ Error leyendo sensor");
        }
    }
}

// Tarea de micro-ROS
void micro_ros_task(void * arg)
{
    allocator = rcl_get_default_allocator();

    // Esperar conexión con el micro-ROS Agent (ejecutándose en el PC)
    ESP_LOGI(TAG, "🔍 Esperando conexión con micro-ROS Agent en PC...");
    ESP_LOGI(TAG, "💡 Asegúrate de ejecutar en el PC:");
    ESP_LOGI(TAG, "   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0");
    
    // Intentar conectar con reintentos
    const int ping_timeout_ms = 1000;
    const int max_attempts = 10;
    int attempts = 0;
    
    while (rmw_uros_ping_agent(ping_timeout_ms, max_attempts) != RCL_RET_OK) {
        ESP_LOGW(TAG, "⏳ Esperando agente... intento %d/%d", ++attempts, max_attempts);
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (attempts >= max_attempts) {
            attempts = 0; // Reiniciar contador
        }
    }

    ESP_LOGI(TAG, "✅ Conectado al micro-ROS Agent!");

    // Crear soporte micro-ROS
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Crear nodo ROS
    RCCHECK(rclc_node_init_default(&node, "esp32_temp_sensor", "", &support));
    ESP_LOGI(TAG, "✅ Nodo ROS creado: 'esp32_temp_sensor'");

    // Crear publicador en el tópico /temperatura
    RCCHECK(rclc_publisher_init_default(
        &temperature_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "temperatura"));
    ESP_LOGI(TAG, "✅ Publicador creado en tópico: /temperatura");

    // Crear timer para publicar cada 2 segundos
    rcl_timer_t timer;
    const unsigned int timer_timeout = 2000; // milisegundos
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true)); // last argument is whether the timer is reset_timer (true) or one-shot (false)
    ESP_LOGI(TAG, "⏱️ Timer configurado: publicación cada %d ms", timer_timeout);

    // Crear executor (gestor de callbacks)
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    ESP_LOGI(TAG, "🚀 Sistema iniciado. Publicando temperatura cada %d segundos...", timer_timeout/1000);
    ESP_LOGI(TAG, "📊 En el PC ejecuta: ros2 topic echo /temperatura");
    
    // Loop principal - ejecutar callbacks
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // Cleanup (nunca se alcanza en este ejemplo)
    RCCHECK(rcl_publisher_fini(&temperature_publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   Sensor DS18B20 con micro-ROS              ║");
    ESP_LOGI(TAG, "║   ESP32 como Nodo ROS                       ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");
    
    // ========================================
    // PARTE 1: Configurar sensor DS18B20
    // ========================================
    ESP_LOGI(TAG, "🔧 Configurando sensor DS18B20...");
    
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = ONE_WIRE_GPIO,
    };
    
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10,
    };

    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
    ESP_LOGI(TAG, "✅ Bus OneWire configurado en GPIO %d", ONE_WIRE_GPIO);

    // Buscar sensor en el bus
    ds18b20_config_t ds_cfg = {}; 
    ESP_LOGI(TAG, "🔍 Buscando sensor DS18B20...");
    
    if (ds18b20_new_device_from_bus(bus, &ds_cfg, &ds18b20s) == ESP_OK) {
        ESP_LOGI(TAG, "✅ Sensor DS18B20 encontrado y listo");
        
        // Probar lectura inicial
        ds18b20_trigger_temperature_conversion(ds18b20s);
        vTaskDelay(pdMS_TO_TICKS(800));
        float temp_test;
        if (ds18b20_get_temperature(ds18b20s, &temp_test) == ESP_OK) {
            ESP_LOGI(TAG, "🌡️ Lectura inicial: %.2f °C", temp_test);
        }
    } else {
        ESP_LOGE(TAG, "❌ Error: No se detectó el sensor DS18B20");
        ESP_LOGE(TAG, "   Revisa las conexiones:");
        ESP_LOGE(TAG, "   - VCC → 3.3V");
        ESP_LOGE(TAG, "   - GND → GND");
        ESP_LOGE(TAG, "   - DATA → GPIO %d", ONE_WIRE_GPIO);
        ESP_LOGE(TAG, "   - Resistencia pull-up 4.7kΩ entre DATA y VCC");
        return; // Salir si no hay sensor
    }

    // Dar tiempo para estabilización
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ========================================
    // PARTE 2: Iniciar micro-ROS
    // ========================================
    ESP_LOGI(TAG, "🌐 Iniciando micro-ROS...");
    
    // Crear tarea de micro-ROS (prioridad 5, 16KB stack)
    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                16000,
                NULL,
                5,
                NULL);
    
    ESP_LOGI(TAG, "✅ Tarea micro-ROS iniciada");
}