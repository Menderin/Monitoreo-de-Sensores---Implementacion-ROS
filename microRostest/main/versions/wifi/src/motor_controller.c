/**
 * @file motor_controller.c
 * @brief Implementación del controlador de motor DC con PWM directo en IN1/IN2
 */

#include "../include/motor_controller.h"
#include "../include/config.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

static bool initialized = false;
static uint8_t current_speed = MOTOR_SPEED_DEFAULT;

// ========================================
// FUNCIONES PÚBLICAS
// ========================================

bool motor_controller_init(void)
{
    ESP_LOGI(TAG, "Inicializando controlador de motor DC con PWM...");
    
    // Configurar timer PWM (compartido por ambos canales)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = MOTOR_PWM_MODE,
        .timer_num        = MOTOR_PWM_TIMER,
        .duty_resolution  = MOTOR_PWM_RESOLUTION,
        .freq_hz          = MOTOR_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    
    if (ledc_timer_config(&ledc_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando timer PWM");
        return false;
    }
    
    // Configurar canal PWM para IN1 (izquierda)
    ledc_channel_config_t ledc_ch_in1 = {
        .speed_mode     = MOTOR_PWM_MODE,
        .channel        = MOTOR_IN1_CHANNEL,
        .timer_sel      = MOTOR_PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_IN1_PIN,
        .duty           = 0,  // Iniciar detenido
        .hpoint         = 0
    };
    
    if (ledc_channel_config(&ledc_ch_in1) != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando canal PWM IN1");
        return false;
    }
    
    // Configurar canal PWM para IN2 (derecha)
    ledc_channel_config_t ledc_ch_in2 = {
        .speed_mode     = MOTOR_PWM_MODE,
        .channel        = MOTOR_IN2_CHANNEL,
        .timer_sel      = MOTOR_PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_IN2_PIN,
        .duty           = 0,  // Iniciar detenido
        .hpoint         = 0
    };
    
    if (ledc_channel_config(&ledc_ch_in2) != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando canal PWM IN2");
        return false;
    }
    
    ESP_LOGI(TAG, "   IN1 (GPIO %d): PWM configurado", MOTOR_IN1_PIN);
    ESP_LOGI(TAG, "   IN2 (GPIO %d): PWM configurado", MOTOR_IN2_PIN);
    ESP_LOGI(TAG, "   Frecuencia PWM: %d Hz", MOTOR_PWM_FREQ);
    ESP_LOGI(TAG, "   Velocidad por defecto: %d/255 (%.0f%%)", 
             current_speed, (current_speed / 255.0) * 100);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  MOTOR CONTROLLER INICIALIZADO");
    ESP_LOGI(TAG, "  PWM directo en IN1/IN2");
    ESP_LOGI(TAG, "========================================");
    
    initialized = true;
    return true;
}

void motor_move_left(uint8_t speed)
{
    if (!initialized) {
        ESP_LOGW(TAG, "Motor no inicializado");
        return;
    }
    
    // Izquierda: IN1=PWM, IN2=0
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_IN1_CHANNEL, speed);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_IN1_CHANNEL);
    
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_IN2_CHANNEL, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_IN2_CHANNEL);
    
    ESP_LOGI(TAG, "⬅️  Motor IZQUIERDA - Velocidad: %d/255 (%.0f%%)", 
             speed, (speed / 255.0) * 100);
}

void motor_move_right(uint8_t speed)
{
    if (!initialized) {
        ESP_LOGW(TAG, "Motor no inicializado");
        return;
    }
    
    // Derecha: IN1=0, IN2=PWM
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_IN1_CHANNEL, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_IN1_CHANNEL);
    
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_IN2_CHANNEL, speed);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_IN2_CHANNEL);
    
    ESP_LOGI(TAG, "➡️  Motor DERECHA - Velocidad: %d/255 (%.0f%%)", 
             speed, (speed / 255.0) * 100);
}

void motor_stop(void)
{
    if (!initialized) {
        return;
    }
    
    // Detener: ambos canales a 0
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_IN1_CHANNEL, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_IN1_CHANNEL);
    
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_IN2_CHANNEL, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_IN2_CHANNEL);
    
    ESP_LOGI(TAG, "⏸️  Motor DETENIDO");
}

void motor_set_speed(uint8_t speed)
{
    current_speed = speed;
    ESP_LOGI(TAG, "🎚️  Velocidad configurada: %d/255 (%.0f%%)", 
             speed, (speed / 255.0) * 100);
}

uint8_t motor_get_speed(void)
{
    return current_speed;
}
