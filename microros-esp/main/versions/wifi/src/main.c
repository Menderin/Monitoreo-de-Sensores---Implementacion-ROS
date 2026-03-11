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
    
    // Inicializar micro-ROS (espera activamente al Agente internamente).
    // El bucle aquí es solo red de seguridad ante fallos post-handshake.
    while (!ros_publisher_init()) {
        ESP_LOGW(TAG, "[Boot] ROS init fallido (fallo interno). Reintentando en %d ms...",
                 ROS_AGENT_REINIT_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(ROS_AGENT_REINIT_DELAY_MS));
    }

    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PUBLICANDO DATOS DE SENSORES");
    ESP_LOGI(TAG, "  Intervalo: %d ms", PUBLISH_INTERVAL_MS);
    ESP_LOGI(TAG, "========================================");
    
    // Loop principal: priorizar procesamiento de comandos de motor
    sensor_data_t data;
    uint32_t last_publish_time = 0;
    uint32_t last_ping_time    = 0;

    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // ── Chequeo periódico de sesión con el Agente ───────────────
        // Se ejecuta cada ROS_AGENT_PING_INTERVAL_MS (5 s).
        // Si falla hace hot-reload internamente; si el reinit también falla,
        // simplemente lo intentará de nuevo en el próximo ciclo.
        if (current_time - last_ping_time >= ROS_AGENT_PING_INTERVAL_MS) {
            if (!ros_agent_check_and_reconnect()) {
                ESP_LOGW(TAG, "Hot-reload falló. Se reintentará en el próximo ciclo.");
            }
            last_ping_time = current_time;
            // Actualizar current_time por si el hot-reload tomó tiempo
            current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }

        // ── Procesar comandos de motor (baja latencia) ───────────────
        ros_executor_spin_some(RCL_MS_TO_NS(10));  // 10ms timeout

        // ── Publicar sensores cada PUBLISH_INTERVAL_MS ───────────────
        if (current_time - last_publish_time >= PUBLISH_INTERVAL_MS) {
            data.temperature    = sensor_read_temperature();
            data.ph             = sensor_read_ph();
            data.voltage_raw_ph = sensor_read_ph_voltage_raw();

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
    ESP_LOGI(TAG, "Paso 1/4: Inicializando WiFi...");
    if (!network_manager_init()) {
        ESP_LOGE(TAG, "Error al inicializar red. Abortando.");
        return;
    }

    // 1.5. Lanzar monitor de conexión WiFi (detecta caídas y reconecta)
    ESP_LOGI(TAG, "Paso 1.5/4: Iniciando monitor WiFi activo...");
    network_manager_start_monitor();
    
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
