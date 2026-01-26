/**
 * @file ros_publisher.c
 * @brief Implementación del publicador micro-ROS
 */

#include "../include/ros_publisher.h"
#include "../include/config.h"
#include "../wifi_config.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "freertos/task.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
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
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  SISTEMA ROS LISTO PARA PUBLICAR");
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

void ros_publisher_deinit(void)
{
    if (!initialized) {
        return;
    }
    
    // Limpiar publicador
    rcl_publisher_fini(&sensor_data_publisher, &node);
    
    // Limpiar nodo
    rcl_node_fini(&node);
    
    // Limpiar soporte
    rclc_support_fini(&support);
    
    initialized = false;
    ESP_LOGI(TAG, "Sistema ROS detenido");
}
