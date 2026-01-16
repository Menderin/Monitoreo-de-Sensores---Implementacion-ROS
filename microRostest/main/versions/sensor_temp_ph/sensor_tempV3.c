#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
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

#include "esp32_serial_transport.h"
#include <driver/uart.h>

// ========================================
// CONFIGURACIÓN DEL SENSOR CWT-BL
// ========================================
// IMPORTANTE: Ajusta estos pines según tu conexión física
#define ADC_TEMP_CHANNEL    ADC_CHANNEL_3  // GPIO39 (VN)  - Temperatura
#define ADC_PH_CHANNEL      ADC_CHANNEL_0  // GPIO36 (VP)  - pH

// Configuración ADC
#define ADC_ATTEN           ADC_ATTEN_DB_11  // 0-3.3V (máximo rango)
#define ADC_WIDTH           ADC_BITWIDTH_12  // 12 bits (0-4095)

// Calibración de temperatura (ajusta según mediciones reales)
#define TEMP_OFFSET_CAL     -1.5  // Offset de calibración en °C (ajustable)

// Calibración de pH - Regresión lineal con soluciones buffer
// Calibración realizada: 15 enero 2026
// Puntos de calibración (valores ADC ESP32):
//   pH 4.01 →  914 mV
//   pH 6.86 → 1701 mV
//   pH 9.18 → 2292 mV
// Fórmula: pH = m × V_mV + b
#define PH_SLOPE            0.00375   // Pendiente (m)
#define PH_INTERCEPT        0.58      // Intercepto (b)

// Configuración micro-ROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGW(TAG, "Failed status on line %d: %d. Continuing.",__LINE__,(int)temp_rc);}}

// Variables globales micro-ROS
rcl_publisher_t temperature_publisher;
rcl_publisher_t ph_publisher;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
std_msgs__msg__Float32 temp_msg;
std_msgs__msg__Float32 ph_msg;

// Variables del ADC
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle;

static const char *TAG = "MICRO_ROS_CWT_BL";
static size_t microros_uart_port = UART_NUM_0;

// ========================================
// FUNCIONES DE CONVERSIÓN
// ========================================

/**
 * @brief Convierte voltaje a temperatura según especificaciones del CWT-BL
 * Fórmula del fabricante: Temperatura = Voltaje(V) × 20.0 - 20.0
 * Con divisor de tensión: 5V sensor → 3.3V ESP32
 * Rango completo: -20°C a 80°C
 */
float voltage_to_temperature(int voltage_mv) {
    // Compensar divisor de tensión: voltaje_real = voltaje_leído × (5.0/3.3)
    float voltage_real_v = (voltage_mv / 1000.0) * (5.0 / 3.3);
    
    // Aplicar fórmula del fabricante: Temp = V × 20.0 - 20.0
    float temp = voltage_real_v * 20.0 - 20.0;
    
    // Aplicar offset de calibración
    temp += TEMP_OFFSET_CAL;
    
    return temp;
}

/**
 * @brief Convierte voltaje a pH
 * 
 * Con divisor de tensión 5V→3.3V para ESP32:
 *   0 mV = 0 pH
 *   3300 mV = 14 pH
 *   pH = V_mV × (14/3300) = V_mV × 0.00424
 * 
 * Voltajes esperados (con divisor):
 *   pH 4.01 →  944 mV
 *   pH 6.86 → 1617 mV
 *   pH 4.01 →  900 mV
 *   pH 6.86 → 1650 mV
 *   pH 9.18 → 2245 mV
 */
float voltage_to_ph(int voltage_mv) {
    // Regresión lineal: pH = m × V_mV + b
    float ph = (voltage_mv * PH_SLOPE) + PH_INTERCEPT;
    
    // Limitar a rango válido 0-14
    if (ph < 0.0) ph = 0.0;
    if (ph > 14.0) ph = 14.0;
    
    return ph;
}

// ========================================
// CONFIGURACIÓN E INICIALIZACIÓN ADC
// ========================================

void init_adc(void) {
    ESP_LOGI(TAG, "🔧 Configurando ADC para sensor CWT-BL...");
    
    // Configurar unidad ADC1
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Configurar canal de temperatura
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_TEMP_CHANNEL, &config));
    ESP_LOGI(TAG, "✅ Canal temperatura configurado en GPIO39");

    // Configurar canal de pH
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_PH_CHANNEL, &config));
    ESP_LOGI(TAG, "✅ Canal pH configurado en GPIO36");

    // Configurar calibración ADC
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        ESP_LOGI(TAG, "✅ Calibración ADC activada (curve fitting)");
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        ESP_LOGI(TAG, "✅ Calibración ADC activada (line fitting)");
    }
#endif
}

// ========================================
// LECTURA DE SENSORES
// ========================================

float read_temperature(void) {
    int adc_raw = 0;
    int voltage_mv = 0;
    
    // Leer valor ADC raw
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_TEMP_CHANNEL, &adc_raw));
    
    // Convertir a voltaje (mV)
    if (adc_cali_handle != NULL) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv));
    } else {
        // Sin calibración: conversión aproximada
        voltage_mv = (adc_raw * 3300) / 4095;
    }
    
    // Convertir voltaje a temperatura
    float temperature = voltage_to_temperature(voltage_mv);
    
    ESP_LOGD(TAG, "Temp: RAW=%d, V=%dmV, T=%.2f°C", adc_raw, voltage_mv, temperature);
    return temperature;
}

float read_ph(void) {
    int adc_raw = 0;
    int voltage_mv = 0;
    
    // Leer valor ADC raw
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_PH_CHANNEL, &adc_raw));
    
    // Convertir a voltaje (mV)
    if (adc_cali_handle != NULL) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv));
    } else {
        // Sin calibración: conversión aproximada
        voltage_mv = (adc_raw * 3300) / 4095;
    }
    
    // Convertir voltaje a pH
    float ph = voltage_to_ph(voltage_mv);
    
    // Log detallado para calibración
    ESP_LOGI(TAG, "🔬 pH: RAW=%d | V=%dmV | pH=%.2f | Ref: 4.01@914mV, 6.86@1701mV, 9.18@2292mV", 
             adc_raw, voltage_mv, ph);
    return ph;
}

// ========================================
// CALLBACK DEL TIMER - PUBLICACIÓN ROS
// ========================================

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer != NULL) {
        // Leer temperatura
        float temperature = read_temperature();
        temp_msg.data = temperature;
        RCSOFTCHECK(rcl_publish(&temperature_publisher, &temp_msg, NULL));
        
        // Leer pH (con valores debug)
        int adc_raw_ph = 0;
        int voltage_mv_ph = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_PH_CHANNEL, &adc_raw_ph));
        if (adc_cali_handle != NULL) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw_ph, &voltage_mv_ph));
        } else {
            voltage_mv_ph = (adc_raw_ph * 3300) / 4095;
        }
        float ph = voltage_to_ph(voltage_mv_ph);
        ph_msg.data = ph;
        RCSOFTCHECK(rcl_publish(&ph_publisher, &ph_msg, NULL));
        
        // Log para debug CON VALORES RAW Y VOLTAJE
        ESP_LOGI(TAG, "📊 RAW=%d | V=%dmV | Temp: %.2f°C | pH: %.2f", adc_raw_ph, voltage_mv_ph, temperature, ph);
    }
}

// ========================================
// TAREA MICRO-ROS
// ========================================

void micro_ros_task(void * arg)
{
    allocator = rcl_get_default_allocator();

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *)&microros_uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );
#else
    ESP_LOGE(TAG, "micro-ROS transport is not set to custom. Update colcon.meta and rebuild.");
    vTaskDelete(NULL);
#endif

    // Esperar conexión con el micro-ROS Agent
    ESP_LOGI(TAG, "🔍 Esperando conexión con micro-ROS Agent en PC...");
    ESP_LOGI(TAG, "💡 Ejecuta en el PC: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0");
    
    const int ping_timeout_ms = 1000;
    const int max_attempts = 10;
    int attempts = 0;
    
    while (rmw_uros_ping_agent(ping_timeout_ms, max_attempts) != RCL_RET_OK) {
        ESP_LOGW(TAG, "⏳ Esperando agente... intento %d/%d", ++attempts, max_attempts);
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (attempts >= max_attempts) {
            attempts = 0;
        }
    }

    ESP_LOGI(TAG, "✅ Conectado al micro-ROS Agent!");

    // Crear soporte micro-ROS
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Crear nodo ROS
    RCCHECK(rclc_node_init_default(&node, "esp32_cwt_bl_sensor", "", &support));
    ESP_LOGI(TAG, "✅ Nodo ROS creado: 'esp32_cwt_bl_sensor'");

    // Crear publicador de temperatura
    RCCHECK(rclc_publisher_init_default(
        &temperature_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "temperatura"));
    ESP_LOGI(TAG, "✅ Publicador creado: /temperatura");

    // Crear publicador de pH
    RCCHECK(rclc_publisher_init_default(
        &ph_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "ph"));
    ESP_LOGI(TAG, "✅ Publicador creado: /ph");

    // Crear timer para publicar cada 2 segundos
    rcl_timer_t timer;
    const unsigned int timer_timeout = 4000;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true));
    ESP_LOGI(TAG, "⏱️ Timer configurado: publicación cada %d ms", timer_timeout);

    // Crear executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    ESP_LOGI(TAG, "🚀 Sistema iniciado. Publicando cada %d segundos...", timer_timeout/1000);
    ESP_LOGI(TAG, "📊 Comandos PC:");
    ESP_LOGI(TAG, "   ros2 topic echo /temperatura");
    ESP_LOGI(TAG, "   ros2 topic echo /ph");
    
    // Loop principal
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // Cleanup
    RCCHECK(rcl_publisher_fini(&temperature_publisher, &node));
    RCCHECK(rcl_publisher_fini(&ph_publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

// ========================================
// FUNCIÓN PRINCIPAL
// ========================================

void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   Sensor CWT-BL (pH + Temperatura)          ║");
    ESP_LOGI(TAG, "║   ESP32 como Nodo ROS - Versión 3           ║");
    ESP_LOGI(TAG, "║   GPIO39: Temperatura | GPIO36: pH          ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");
    
    // Inicializar ADC
    init_adc();
    
    // Prueba de lectura inicial
    ESP_LOGI(TAG, "🧪 Prueba de lecturas iniciales...");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    float temp_test = read_temperature();
    float ph_test = read_ph();
    ESP_LOGI(TAG, "🌡️ Temperatura: %.2f °C", temp_test);
    ESP_LOGI(TAG, "🧪 pH: %.2f", ph_test);
    
    // Dar tiempo para estabilización
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Iniciar micro-ROS
    ESP_LOGI(TAG, "🌐 Iniciando micro-ROS...");
    
    xTaskCreate(micro_ros_task,
                "micro_ros_task",
                16000,
                NULL,
                5,
                NULL);
    
    ESP_LOGI(TAG, "✅ Tarea micro-ROS iniciada");
}
