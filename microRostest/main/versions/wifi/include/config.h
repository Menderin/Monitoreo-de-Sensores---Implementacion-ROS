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
#define TEMP_OFFSET_CAL     -0.7

// pH: conversiones anteriores (comentadas para historial)
// v1: slope=0.00375, intercept=0.58
// v2: slope=0.00356, intercept=1.049 (buffers 4.01, 6.86, 9.18 - ene 19)

/*  Puntos de calibración usados: 3
     Voltaje (mV)   pH buffer   pH predicho     Error
  ────────────────────────────────────────────────────────
           915.00       4.010         3.998   +0.0123
          1713.00       6.860         6.888   -0.0280
          2341.50       9.180         9.164   +0.0156
  ────────────────────────────────────────────────────────

  slope     = 0.003622
  intercept = 0.683614
  R²        = 0.999912 */ 

// pH: conversión lineal de mV a pH (calibrado 2026-01-20)
// Buffers: pH 4.01→878mV, 6.82→1687.5mV, 9.18→2311mV
// pH: calibrado 2026-02-19                                 
// Buffers: pH 4.01→915mV, pH 6.86→1713mV, pH 9.18→2342mV    
#define PH_SLOPE       0.003622                              
#define PH_INTERCEPT   0.683614 

// ========================================
// CONFIGURACIÓN DE PUBLICACIÓN
// ========================================

#define PUBLISH_INTERVAL_MS 4000

// ========================================
// CONFIGURACIÓN DE TAREAS
// ========================================

// Optimizado: Stack reducido de 16KB a 8KB (-8KB RAM total)
// CRÍTICO: Mínimo para micro-ROS sin crash
#define MICRO_ROS_STACK_SIZE    8192
#define MICRO_ROS_PRIORITY      5
#define MICRO_ROS_CORE          1  // APP_CPU (Core 1)

// ========================================
// CONFIGURACIÓN DE NODO ROS
// ========================================

#define NODE_NAME               "esp32_sensor_node"
#define TOPIC_TEMPERATURE       "temperatura"
#define TOPIC_PH                "ph"
#define TOPIC_VOLTAGE_RAW_PH    "voltage_raw_ph"
#define TOPIC_MOTOR_CMD         "motor_commands"

// ========================================
// CONFIGURACIÓN DE MOTOR DC - Driver MINI 298
// ========================================

// Pines GPIO para driver MINI 298 (con PWM directo en IN1/IN2)
#define MOTOR_IN1_PIN    GPIO_NUM_25  // PWM dirección izquierda
#define MOTOR_IN2_PIN    GPIO_NUM_26  // PWM dirección derecha

// Configuración PWM para control de velocidad
#define MOTOR_PWM_FREQ       1000           // 1 KHz
#define MOTOR_PWM_TIMER      LEDC_TIMER_0
#define MOTOR_PWM_MODE       LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT  // 0-255

// Canales PWM (uno para cada pin)
#define MOTOR_IN1_CHANNEL    LEDC_CHANNEL_0
#define MOTOR_IN2_CHANNEL    LEDC_CHANNEL_1

// Niveles de velocidad (duty cycle)
#define MOTOR_SPEED_SLOW     102  // 40% (~40% velocidad)
#define MOTOR_SPEED_MEDIUM   178  // 70% (~70% velocidad)
#define MOTOR_SPEED_FAST     255  // 100% (velocidad máxima)

// Velocidad por defecto
#define MOTOR_SPEED_DEFAULT  MOTOR_SPEED_FAST

#endif // CONFIG_H
