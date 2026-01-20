/**
 * @file ros_publisher.c
 * @brief Implementación del publicador micro-ROS
 */

#include "../include/ros_publisher.h"
#include "../include/config.h"
#include "../wifi_config.h"

#include "esp_log.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
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

// ========================================
// VARIABLES PRIVADAS
// ========================================

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;

static rcl_publisher_t temperature_publisher;
static rcl_publisher_t ph_publisher;
static rcl_publisher_t voltage_raw_ph_publisher;

static std_msgs__msg__Float32 temperature_msg;
static std_msgs__msg__Float32 ph_msg;
static std_msgs__msg__Float32 voltage_raw_ph_msg;

static bool initialized = false;

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
    
    // Crear nodo
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    ESP_LOGI(TAG, "Nodo creado: '%s'", NODE_NAME);
    
    // Crear publicador de temperatura
    RCCHECK(rclc_publisher_init_default(
        &temperature_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        TOPIC_TEMPERATURE));
    ESP_LOGI(TAG, "Publicador creado: '/%s'", TOPIC_TEMPERATURE);
    
    // Crear publicador de pH
    RCCHECK(rclc_publisher_init_default(
        &ph_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        TOPIC_PH));
    ESP_LOGI(TAG, "Publicador creado: '/%s'", TOPIC_PH);
    
    // Crear publicador de voltaje raw pH (para calibración)
    RCCHECK(rclc_publisher_init_default(
        &voltage_raw_ph_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        TOPIC_VOLTAGE_RAW_PH));
    ESP_LOGI(TAG, "Publicador creado: '/%s' (calibración)", TOPIC_VOLTAGE_RAW_PH);
    
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
    
    // Actualizar mensajes
    temperature_msg.data = data->temperature;
    ph_msg.data = data->ph;
    voltage_raw_ph_msg.data = data->voltage_raw_ph;
    
    // Publicar
    RCSOFTCHECK(rcl_publish(&temperature_publisher, &temperature_msg, NULL));
    RCSOFTCHECK(rcl_publish(&ph_publisher, &ph_msg, NULL));
    RCSOFTCHECK(rcl_publish(&voltage_raw_ph_publisher, &voltage_raw_ph_msg, NULL));
    
    ESP_LOGI(TAG, "Temp: %.2f °C | pH: %.2f | Raw: %.2f mV", 
             data->temperature, data->ph, data->voltage_raw_ph);
    
    return true;
}

void ros_publisher_deinit(void)
{
    if (!initialized) {
        return;
    }
    
    // Limpiar publicadores
    rcl_publisher_fini(&temperature_publisher, &node);
    rcl_publisher_fini(&ph_publisher, &node);
    rcl_publisher_fini(&voltage_raw_ph_publisher, &node);
    
    // Limpiar nodo
    rcl_node_fini(&node);
    
    // Limpiar soporte
    rclc_support_fini(&support);
    
    initialized = false;
    ESP_LOGI(TAG, "Sistema ROS detenido");
}
