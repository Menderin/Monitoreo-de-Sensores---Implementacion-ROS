/**
 * @file main.c
 * @brief Aplicación principal - Sensor CWT-BL con micro-ROS sobre WiFi/UDP
 * 
 * Arquitectura modular con separación de responsabilidades:
 * - sensor_manager: Gestión de ADC y sensores
 * - ros_publisher: Comunicación micro-ROS
 * - network_manager: Gestión de WiFi
 * 
 * @author ESP32 micro-ROS Team
 * @date 2026
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "../include/config.h"
#include "../include/sensor_manager.h"
#include "../include/ros_publisher.h"
#include "../include/network_manager.h"
#include "../include/motor_controller.h"

#include <rcl/rcl.h>

static const char *TAG = "MAIN";

// ========================================
// TAREA MICRO-ROS
// ========================================

void micro_ros_task(void *arg)
{
    ESP_LOGI(TAG, "Iniciando tarea micro-ROS...");
    
    // Inicializar micro-ROS y crear publicadores
    if (!ros_publisher_init()) {
        ESP_LOGE(TAG, "Error al inicializar ROS publisher");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PUBLICANDO DATOS DE SENSORES");
    ESP_LOGI(TAG, "  Intervalo: %d ms", PUBLISH_INTERVAL_MS);
    ESP_LOGI(TAG, "========================================");
    
    // Loop principal: priorizar procesamiento de comandos de motor
    sensor_data_t data;
    uint32_t last_publish_time = 0;
    
    while (1) {
        // CRÍTICO: Procesar comandos de motor frecuentemente (baja latencia)
        ros_executor_spin_some(RCL_MS_TO_NS(10));  // 10ms timeout
        
        // Publicar sensores solo cada PUBLISH_INTERVAL_MS
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_publish_time >= PUBLISH_INTERVAL_MS) {
            // Leer sensores
            data.temperature = sensor_read_temperature();
            data.ph = sensor_read_ph();
            data.voltage_raw_ph = sensor_read_ph_voltage_raw();
            
            // Publicar datos
            ros_publisher_publish(&data);
            
            last_publish_time = current_time;
        }
        
        // Espera mínima entre iteraciones (50ms = ~20Hz para motor)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Cleanup (nunca se alcanza en operación normal)
    ros_publisher_deinit();
    vTaskDelete(NULL);
}

// ========================================
// FUNCIÓN PRINCIPAL
// ========================================

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "   SENSOR CWT-BL (pH + Temperatura + Motor)");
    ESP_LOGI(TAG, "   ESP32 + WiFi + micro-ROS (UDP)");
    ESP_LOGI(TAG, "   Arquitectura Modular con Control de Motor");
    ESP_LOGI(TAG, "==============================================");
    
    // 1. Inicializar conexión WiFi
    ESP_LOGI(TAG, "Paso 1/3: Inicializando WiFi...");
    if (!network_manager_init()) {
        ESP_LOGE(TAG, "Error al inicializar red. Abortando.");
        return;
    }
    
    // 2. Inicializar sistema de sensores
    ESP_LOGI(TAG, "Paso 2/3: Inicializando sensores...");
    if (!sensor_manager_init()) {
        ESP_LOGE(TAG, "Error al inicializar sensores. Abortando.");
        return;
    }
    
    // Prueba rápida de sensores
    ESP_LOGI(TAG, "Prueba de sensores:");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "   Temperatura: %.2f °C", sensor_read_temperature());
    ESP_LOGI(TAG, "   pH: %.2f", sensor_read_ph());
    
    // 2.5. Inicializar controlador de motor
    ESP_LOGI(TAG, "Paso 2.5/4: Inicializando controlador de motor...");
    if (!motor_controller_init()) {
        ESP_LOGW(TAG, "Advertencia: Motor controller no inicializado. Continuando...");
    }
    
    // 3. Iniciar tarea micro-ROS
    ESP_LOGI(TAG, "Paso 3/4: Iniciando publicador micro-ROS...");
    xTaskCreatePinnedToCore(
        micro_ros_task,              // Función
        "micro_ros_task",            // Nombre
        MICRO_ROS_STACK_SIZE,        // Stack size
        NULL,                        // Parámetro
        MICRO_ROS_PRIORITY,          // Prioridad
        NULL,                        // Handle
        MICRO_ROS_CORE);             // Core (1 = APP_CPU)
    
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "   SISTEMA INICIADO CORRECTAMENTE");
    ESP_LOGI(TAG, "==============================================");
}
