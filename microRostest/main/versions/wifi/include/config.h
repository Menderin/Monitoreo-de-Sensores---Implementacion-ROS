/**
 * @file config.h
 * @brief Configuración centralizada del sistema de sensores
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "esp_adc/adc_oneshot.h"

// ========================================
// CONFIGURACIÓN DEL HARDWARE
// ========================================

// Canales ADC
#define ADC_TEMP_CHANNEL    ADC_CHANNEL_3  // GPIO39 (VN) - Temperatura
#define ADC_PH_CHANNEL      ADC_CHANNEL_0  // GPIO36 (VP) - pH
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define ADC_WIDTH           ADC_BITWIDTH_12

// ========================================
// CALIBRACIÓN DE SENSORES
// ========================================

// Temperatura: conversión de voltaje a grados Celsius
#define TEMP_OFFSET_CAL     -1.5

// pH: conversión lineal de mV a pH
#define PH_SLOPE            0.00375
#define PH_INTERCEPT        0.58

// ========================================
// CONFIGURACIÓN DE PUBLICACIÓN
// ========================================

#define PUBLISH_INTERVAL_MS 4000

// ========================================
// CONFIGURACIÓN DE TAREAS
// ========================================

#define MICRO_ROS_STACK_SIZE    16384
#define MICRO_ROS_PRIORITY      5
#define MICRO_ROS_CORE          1  // APP_CPU (Core 1)

// ========================================
// CONFIGURACIÓN DE NODO ROS
// ========================================

#define NODE_NAME               "esp32_sensor_node"
#define TOPIC_TEMPERATURE       "temperatura"
#define TOPIC_PH                "ph"

#endif // CONFIG_H
