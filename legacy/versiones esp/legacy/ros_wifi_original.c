/**
 * @file ros_wifi.c
 * @brief Sensor CWT-BL (pH + Temperatura) con micro-ROS sobre WiFi/UDP
 * 
 * Usa la función uros_network_interface_initialize() del componente micro-ROS
 * que se configura automáticamente desde sdkconfig (menuconfig)
 */

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"

// ADC
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Librerías micro-ROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

// Interfaz de red de micro-ROS (maneja WiFi automáticamente)
#include <uros_network_interfaces.h>

// Configuración WiFi (para mostrar info, no para inicializar)
#include "wifi_config.h"

static const char *TAG = "MICRO_ROS_WIFI";

// ========================================
// CONFIGURACIÓN DEL SENSOR CWT-BL
// ========================================
#define ADC_TEMP_CHANNEL    ADC_CHANNEL_3  // GPIO39 (VN)  - Temperatura
#define ADC_PH_CHANNEL      ADC_CHANNEL_0  // GPIO36 (VP)  - pH
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define ADC_WIDTH           ADC_BITWIDTH_12

// Calibración
#define TEMP_OFFSET_CAL     -1.5
#define PH_SLOPE            0.00375
#define PH_INTERCEPT        0.58

// Publicación
#define PUBLISH_INTERVAL_MS 1000
#define RCCHECK_INTERVAL_MS 100

// ========================================
// MACRO DE VERIFICACIÓN
// ========================================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG,"RCCHECK failed at line %d: %d. Aborting.",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGW(TAG,"RCSOFTCHECK failed at line %d: %d",__LINE__,(int)temp_rc);}}

// ========================================
// VARIABLES GLOBALES
// ========================================
static rcl_publisher_t temperature_publisher;
static rcl_publisher_t ph_publisher;
static std_msgs__msg__Float32 temperature_msg;
static std_msgs__msg__Float32 ph_msg;

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// ========================================
// FUNCIONES DE CONVERSIÓN
// ========================================

float voltage_to_temperature(int voltage_mv) {
    float voltage_real_v = (voltage_mv / 1000.0) * (5.0 / 3.3);
    float temp = voltage_real_v * 20.0 - 20.0 + TEMP_OFFSET_CAL;
    return temp;
}

float voltage_to_ph(int voltage_mv) {
    float ph = (voltage_mv * PH_SLOPE) + PH_INTERCEPT;
    if (ph < 0.0) ph = 0.0;
    if (ph > 14.0) ph = 14.0;
    return ph;
}

// ========================================
// INICIALIZACIÓN ADC
// ========================================

void adc_init(void)
{
    ESP_LOGI(TAG, "Configurando ADC...");
    
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_TEMP_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_PH_CHANNEL, &config));
    
    // Calibración
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        ESP_LOGI(TAG, "  Calibracion ADC: OK");
    }
    
    ESP_LOGI(TAG, "  Canal temperatura: GPIO39");
    ESP_LOGI(TAG, "  Canal pH: GPIO36");
}

// ========================================
// LECTURA DE SENSORES
// ========================================

float read_temperature(void)
{
    int raw, voltage_mv;
    adc_oneshot_read(adc_handle, ADC_TEMP_CHANNEL, &raw);
    if (adc_cali_handle) {
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv);
    } else {
        voltage_mv = raw * 3300 / 4095;
    }
    return voltage_to_temperature(voltage_mv);
}

float read_ph(void)
{
    int raw, voltage_mv;
    adc_oneshot_read(adc_handle, ADC_PH_CHANNEL, &raw);
    if (adc_cali_handle) {
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv);
    } else {
        voltage_mv = raw * 3300 / 4095;
    }
    return voltage_to_ph(voltage_mv);
}

// ========================================
// TAREA micro-ROS
// ========================================

void micro_ros_task(void *arg)
{
    allocator = rcl_get_default_allocator();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  CONFIGURACION MICRO-ROS");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Agent IP:   %s", CONFIG_MICRO_ROS_AGENT_IP);
    ESP_LOGI(TAG, "  Agent Port: %s", CONFIG_MICRO_ROS_AGENT_PORT);
    ESP_LOGI(TAG, "========================================");

    // Opciones de inicialización
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    
    // Configurar dirección UDP del agente
    ESP_LOGI(TAG, "Configurando transporte UDP...");
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    ESP_LOGI(TAG, "Transporte UDP configurado");
#endif

    // Crear soporte (esto establece la conexión con el agente)
    ESP_LOGI(TAG, "Conectando con micro-ROS Agent...");
    ESP_LOGI(TAG, "Ejecuta en el PC:");
    ESP_LOGI(TAG, "  ros2 run micro_ros_agent micro_ros_agent udp4 --port %s", CONFIG_MICRO_ROS_AGENT_PORT);
    
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  CONECTADO A MICRO-ROS AGENT!");
    ESP_LOGI(TAG, "========================================");

    // Crear nodo
    RCCHECK(rclc_node_init_default(&node, "esp32_sensor_node", "", &support));
    ESP_LOGI(TAG, "Nodo creado: 'esp32_sensor_node'");

    // Crear publicador de temperatura
    RCCHECK(rclc_publisher_init_default(
        &temperature_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "temperatura"));
    ESP_LOGI(TAG, "Publicador creado: '/temperatura'");

    // Crear publicador de pH
    RCCHECK(rclc_publisher_init_default(
        &ph_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "ph"));
    ESP_LOGI(TAG, "Publicador creado: '/ph'");

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PUBLICANDO DATOS DE SENSORES");
    ESP_LOGI(TAG, "  Intervalo: %d ms", PUBLISH_INTERVAL_MS);
    ESP_LOGI(TAG, "========================================");

    // Loop de publicación
    while (1) {
        // Leer sensores
        temperature_msg.data = read_temperature();
        ph_msg.data = read_ph();
        
        // Publicar
        RCSOFTCHECK(rcl_publish(&temperature_publisher, &temperature_msg, NULL));
        RCSOFTCHECK(rcl_publish(&ph_publisher, &ph_msg, NULL));
        
        ESP_LOGI(TAG, "Temp: %.2f C | pH: %.2f", temperature_msg.data, ph_msg.data);
        
        vTaskDelay(pdMS_TO_TICKS(PUBLISH_INTERVAL_MS));
    }
    
    // Cleanup (nunca llegará aquí en operación normal)
    RCCHECK(rcl_publisher_fini(&temperature_publisher, &node));
    RCCHECK(rcl_publisher_fini(&ph_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
    
    vTaskDelete(NULL);
}

// ========================================
// FUNCIÓN PRINCIPAL
// ========================================

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "   SENSOR CWT-BL (pH + Temperatura)");
    ESP_LOGI(TAG, "   ESP32 + WiFi + micro-ROS (UDP)");
    ESP_LOGI(TAG, "   GPIO39: Temp | GPIO36: pH");
    ESP_LOGI(TAG, "==============================================");
    
    // Inicializar red usando la función del componente micro-ROS
    // Esta función usa CONFIG_ESP_WIFI_SSID y CONFIG_ESP_WIFI_PASSWORD de sdkconfig
    ESP_LOGI(TAG, "Inicializando WiFi (via micro-ROS component)...");
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_LOGI(TAG, "WiFi conectado!");
    
    // Inicializar ADC
    adc_init();
    
    // Prueba de lectura
    ESP_LOGI(TAG, "Prueba de sensores...");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "   Temperatura: %.2f C", read_temperature());
    ESP_LOGI(TAG, "   pH: %.2f", read_ph());
    
    // Iniciar tarea micro-ROS en APP_CPU para que PRO_CPU maneje WiFi
    xTaskCreatePinnedToCore(micro_ros_task, 
                            "micro_ros_task", 
                            16384,  // Stack size
                            NULL,   // Parámetro
                            5,      // Prioridad
                            NULL,   // Handle
                            1);     // Core 1 (APP_CPU)
    
    ESP_LOGI(TAG, "Sistema iniciado correctamente");
}
