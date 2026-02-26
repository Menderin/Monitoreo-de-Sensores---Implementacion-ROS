/**
 * @file ros_publisher.c
 * @brief Implementación del publicador y suscriptor micro-ROS (híbrido)
 */

#include "../include/ros_publisher.h"
#include "../include/motor_controller.h"
#include "../include/config.h"
#include "../wifi_config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>  // Para atoi()

#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "freertos/task.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>

static const char *TAG = "ROS_PUBLISHER";

// ========================================
// MACRO DE VERIFICACIÓN
// ========================================
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)) { \
        ESP_LOGE(TAG,"RCCHECK failed at line %d: %d", __LINE__, (int)temp_rc); \
        return false; \
    } \
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)) { \
        ESP_LOGW(TAG,"RCSOFTCHECK failed at line %d: %d", __LINE__, (int)temp_rc); \
    } \
}

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;

// Publicador único con array de floats
static rcl_publisher_t sensor_data_publisher;
static std_msgs__msg__Float32MultiArray sensor_data_msg;

// Suscriptor para comandos de motor
static rcl_subscription_t motor_cmd_subscriber;
static std_msgs__msg__String motor_cmd_msg;
static char motor_cmd_buffer[16];  // Buffer estático para el string

// Executor para manejar callbacks
static rclc_executor_t executor;

// Optimización: Buffers estáticos preallocados (no fragmentan HEAP)
static float sensor_data_array[5];  // [temp, pH, voltage, mac_part1, mac_part2]
static uint8_t mac_address[6] = {0};

// Preallocación de secuencia para evitar malloc en runtime
static rosidl_runtime_c__float__Sequence data_sequence = {
    .data = NULL,  // Se asigna en init
    .size = 0,
    .capacity = 5
};

static bool initialized = false;

// ========================================
// FUNCIONES PRIVADAS
// ========================================

static void get_device_mac_floats(float *mac_part1, float *mac_part2) {
    // Leer MAC una sola vez
    if (mac_address[0] == 0) {
        esp_read_mac(mac_address, ESP_MAC_WIFI_STA);
    }
    
    // Dividir MAC en 2 partes de 24 bits cada una
    // MAC: AA:BB:CC:DD:EE:FF -> 0xAABBCC y 0xDDEEFF
    uint32_t part1 = (mac_address[0] << 16) | (mac_address[1] << 8) | mac_address[2];
    uint32_t part2 = (mac_address[3] << 16) | (mac_address[4] << 8) | mac_address[5];
    
    *mac_part1 = (float)part1;
    *mac_part2 = (float)part2;
}

// ========================================
// CALLBACKS
// ========================================

static void motor_cmd_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    
    if (msg->data.data == NULL || msg->data.size == 0) {
        ESP_LOGW(TAG, "Comando de motor vacío recibido");
        return;
    }
    
    ESP_LOGI(TAG, "📩 Comando recibido: '%s'", msg->data.data);
    
    // Obtener velocidad actual configurada
    uint8_t speed = motor_get_speed();
    
    // Comparar comando y ejecutar acción
    if (strcmp(msg->data.data, "LEFT") == 0) {
        motor_move_left(speed);
    }
    else if (strcmp(msg->data.data, "RIGHT") == 0) {
        motor_move_right(speed);
    }
    else if (strcmp(msg->data.data, "STOP") == 0) {
        motor_stop();
    }
    // Comando de velocidad con formato SPEED_SET_XX (donde XX es porcentaje 0-100)
    else if (strncmp(msg->data.data, "SPEED_SET_", 10) == 0) {
        // Parsear porcentaje después de "SPEED_SET_"
        int percent = atoi(msg->data.data + 10);
        
        // Validar rango
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
        
        // Convertir porcentaje a duty cycle (0-255)
        uint8_t duty = (uint8_t)((percent * 255) / 100);
        
        motor_set_speed(duty);
        ESP_LOGI(TAG, "🎚️  Velocidad configurada: %d%% (duty=%d/255)", percent, duty);
    }
    // Comandos de velocidad legacy (compatibilidad)
    else if (strcmp(msg->data.data, "SPEED_SLOW") == 0) {
        motor_set_speed(MOTOR_SPEED_SLOW);
    }
    else if (strcmp(msg->data.data, "SPEED_MEDIUM") == 0) {
        motor_set_speed(MOTOR_SPEED_MEDIUM);
    }
    else if (strcmp(msg->data.data, "SPEED_FAST") == 0) {
        motor_set_speed(MOTOR_SPEED_FAST);
    }
    else {
        ESP_LOGW(TAG, "Comando desconocido: '%s'", msg->data.data);
    }
}

// ========================================
// FUNCIONES PÚBLICAS
// ========================================

bool ros_publisher_init(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  INICIALIZANDO MICRO-ROS");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Agent IP:   %s", CONFIG_MICRO_ROS_AGENT_IP);
    ESP_LOGI(TAG, "Agent Port: %s", CONFIG_MICRO_ROS_AGENT_PORT);
    ESP_LOGI(TAG, "========================================");
    
    allocator = rcl_get_default_allocator();
    
    // Inicializar opciones
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    
    // Configurar transporte UDP
    ESP_LOGI(TAG, "Configurando transporte UDP...");
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP, 
        CONFIG_MICRO_ROS_AGENT_PORT, 
        rmw_options));
    ESP_LOGI(TAG, "Transporte UDP configurado");
#endif
    
    // Crear soporte y establecer conexión
    ESP_LOGI(TAG, "Conectando con micro-ROS Agent...");
    ESP_LOGI(TAG, "Ejecuta en el PC:");
    ESP_LOGI(TAG, "  ros2 run micro_ros_agent micro_ros_agent udp4 --port %s", 
             CONFIG_MICRO_ROS_AGENT_PORT);
    
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  CONECTADO A MICRO-ROS AGENT!");
    ESP_LOGI(TAG, "========================================");
    
    // Optimización AGRESIVA: Liberar máxima memoria WiFi
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // Power save mínimo
    vTaskDelay(pdMS_TO_TICKS(500));  // Delay mayor para estabilizar
    
    // Debug: Mostrar memoria disponible
    ESP_LOGI(TAG, "HEAP libre antes de crear nodo: %lu bytes", esp_get_free_heap_size());
    
    // Leer MAC para nombre único de nodo
    if (mac_address[0] == 0) {
        esp_read_mac(mac_address, ESP_MAC_WIFI_STA);
    }
    
    // Crear nombre de nodo dinámico para evitar colisiones DDS
    char node_name[32];
    snprintf(node_name, sizeof(node_name), "esp32_%02X%02X%02X", 
             mac_address[3], mac_address[4], mac_address[5]);
    
    // Crear nodo con nombre único
    RCCHECK(rclc_node_init_default(&node, node_name, "", &support));
    ESP_LOGI(TAG, "Nodo creado: '%s' (ID único basado en MAC)", node_name);
    ESP_LOGI(TAG, "HEAP libre después de nodo: %lu bytes", esp_get_free_heap_size());
    
    // Crear publicador único para sensor_data (Float32MultiArray)
    RCCHECK(rclc_publisher_init_default(
        &sensor_data_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "sensor_data"));
    ESP_LOGI(TAG, "Publicador creado: '/sensor_data' (Float32MultiArray)");
    
    // Optimización: Usar secuencia preallocada
    data_sequence.data = sensor_data_array;
    data_sequence.size = 5;
    data_sequence.capacity = 5;
    
    sensor_data_msg.data = data_sequence;
    
    // Layout (opcional, para metadata) - sin alocar
    sensor_data_msg.layout.dim.data = NULL;
    sensor_data_msg.layout.dim.size = 0;
    sensor_data_msg.layout.dim.capacity = 0;
    sensor_data_msg.layout.data_offset = 0;
    
    ESP_LOGI(TAG, "HEAP libre antes de crear subscriber: %lu bytes", esp_get_free_heap_size());
    
    // Crear suscriptor para comandos de motor
    RCCHECK(rclc_subscription_init_default(
        &motor_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        TOPIC_MOTOR_CMD));
    ESP_LOGI(TAG, "Suscriptor creado: '/%s' (String)", TOPIC_MOTOR_CMD);
    
    // Inicializar buffer del mensaje
    motor_cmd_msg.data.data = motor_cmd_buffer;
    motor_cmd_msg.data.capacity = sizeof(motor_cmd_buffer);
    motor_cmd_msg.data.size = 0;
    
    // Crear executor con 1 handle (el subscriber)
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &motor_cmd_subscriber,
        &motor_cmd_msg,
        &motor_cmd_callback,
        ON_NEW_DATA));
    ESP_LOGI(TAG, "Executor creado con 1 subscription");
    
    ESP_LOGI(TAG, "HEAP libre después de subscriber: %lu bytes", esp_get_free_heap_size());
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  SISTEMA ROS LISTO (PUBLISHER + SUBSCRIBER)");
    ESP_LOGI(TAG, "========================================");
    
    initialized = true;
    return true;
}

bool ros_publisher_publish(const sensor_data_t *data)
{
    if (!initialized) {
        ESP_LOGE(TAG, "ROS publisher no inicializado");
        return false;
    }
    
    if (data == NULL) {
        ESP_LOGE(TAG, "Datos de sensores nulos");
        return false;
    }
    
    // Obtener MAC dividida en 2 floats
    float mac_part1, mac_part2;
    get_device_mac_floats(&mac_part1, &mac_part2);
    
    // Llenar array: [temp, pH, voltage, mac_part1, mac_part2]
    sensor_data_array[0] = data->temperature;
    sensor_data_array[1] = data->ph;
    sensor_data_array[2] = data->voltage_raw_ph;
    sensor_data_array[3] = mac_part1;
    sensor_data_array[4] = mac_part2;
    
    // Publicar
    RCSOFTCHECK(rcl_publish(&sensor_data_publisher, &sensor_data_msg, NULL));
    
    ESP_LOGI(TAG, "Publicado: [%.2f, %.2f, %.2f, %.0f, %.0f]", 
             sensor_data_array[0], sensor_data_array[1], sensor_data_array[2],
             sensor_data_array[3], sensor_data_array[4]);
    
    return true;
}

bool ros_executor_spin_some(uint64_t timeout_ns)
{
    if (!initialized) {
        return false;
    }
    
    // Procesar callbacks pendientes (non-blocking)
    rcl_ret_t ret = rclc_executor_spin_some(&executor, timeout_ns);
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
        ESP_LOGW(TAG, "Executor spin returned: %d", (int)ret);
        return false;
    }
    
    return true;
}

void ros_publisher_deinit(void)
{
    if (!initialized) {
        return;
    }
    
    // Limpiar executor
    rclc_executor_fini(&executor);
    
    // Limpiar suscriptor
    RCSOFTCHECK(rcl_subscription_fini(&motor_cmd_subscriber, &node));
    
    // Limpiar publicador
    RCSOFTCHECK(rcl_publisher_fini(&sensor_data_publisher, &node));
    
    // Limpiar nodo
    RCSOFTCHECK(rcl_node_fini(&node));
    
    // Limpiar soporte
    rclc_support_fini(&support);
    
    initialized = false;
    ESP_LOGI(TAG, "Sistema ROS detenido");
}
