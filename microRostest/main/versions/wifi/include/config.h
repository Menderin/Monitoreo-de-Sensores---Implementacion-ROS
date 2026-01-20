/**
 * @file config.h
 * @brief Configuración centralizada del sistema de sensores
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "esp_adc/adc_oneshot.h"

// ========================================
// CONFIGURACIÓN DEL HARDWARE
// =====================================

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

// pH: conversiones anteriores (comentadas para historial)
// v1: slope=0.00375, intercept=0.58
// v2: slope=0.00356, intercept=1.049 (buffers 4.01, 6.86, 9.18 - ene 19)

// pH: conversión lineal de mV a pH (calibrado 2026-01-20)
// Buffers: pH 4.01→878mV, 6.82→1687.5mV, 9.18→2311mV
#define PH_SLOPE            0.003601
#define PH_INTERCEPT        0.849

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
#define TOPIC_VOLTAGE_RAW_PH    "voltage_raw_ph"

#endif // CONFIG_H
