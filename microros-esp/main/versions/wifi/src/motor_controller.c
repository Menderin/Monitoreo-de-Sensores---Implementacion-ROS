/**
 * @file motor_controller.c
 * @brief Controlador de motor DC - Driver L298N
 *
 * Esquema de control:
 *   ENA (GPIO14) → PWM de velocidad (LEDC)
 *   IN1 (GPIO27) → dirección digital (HIGH = izquierda activa)
 *   IN2 (GPIO26) → dirección digital (HIGH = derecha activa)
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
    ESP_LOGI(TAG, "Inicializando controlador de motor DC (ENA+IN1+IN2)...");

    // ── 1. Configurar IN1 e IN2 como GPIO digitales de salida ──────────
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_IN1_PIN) | (1ULL << MOTOR_IN2_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando GPIO IN1/IN2");
        return false;
    }
    // Arrancar con motor detenido: IN1=0, IN2=0
    gpio_set_level(MOTOR_IN1_PIN, 0);
    gpio_set_level(MOTOR_IN2_PIN, 0);

    // ── 2. Configurar timer PWM para ENA ───────────────────────────────
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = MOTOR_PWM_MODE,
        .timer_num       = MOTOR_PWM_TIMER,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz         = MOTOR_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    if (ledc_timer_config(&ledc_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando timer PWM");
        return false;
    }

    // ── 3. Configurar canal PWM para ENA ───────────────────────────────
    ledc_channel_config_t ledc_ch_ena = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel    = MOTOR_ENA_CHANNEL,
        .timer_sel  = MOTOR_PWM_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = MOTOR_ENA_PIN,
        .duty       = 0,   // Iniciar detenido
        .hpoint     = 0
    };
    if (ledc_channel_config(&ledc_ch_ena) != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando canal PWM ENA");
        return false;
    }

    ESP_LOGI(TAG, "   ENA (GPIO %d): PWM %d Hz", MOTOR_ENA_PIN, MOTOR_PWM_FREQ);
    ESP_LOGI(TAG, "   IN1 (GPIO %d): digital", MOTOR_IN1_PIN);
    ESP_LOGI(TAG, "   IN2 (GPIO %d): digital", MOTOR_IN2_PIN);
    ESP_LOGI(TAG, "   Velocidad por defecto: %d/255 (%.0f%%)",
             current_speed, (current_speed / 255.0) * 100);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  MOTOR CONTROLLER INICIALIZADO (L298N)");
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

    // Izquierda: IN1=HIGH, IN2=LOW, ENA=PWM(speed)
    gpio_set_level(MOTOR_IN1_PIN, 1);
    gpio_set_level(MOTOR_IN2_PIN, 0);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_ENA_CHANNEL, speed);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_ENA_CHANNEL);

    ESP_LOGI(TAG, "⬅️  Motor IZQUIERDA - Velocidad: %d/255 (%.0f%%)",
             speed, (speed / 255.0) * 100);
}

void motor_move_right(uint8_t speed)
{
    if (!initialized) {
        ESP_LOGW(TAG, "Motor no inicializado");
        return;
    }

    // Derecha: IN1=LOW, IN2=HIGH, ENA=PWM(speed)
    gpio_set_level(MOTOR_IN1_PIN, 0);
    gpio_set_level(MOTOR_IN2_PIN, 1);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_ENA_CHANNEL, speed);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_ENA_CHANNEL);

    ESP_LOGI(TAG, "➡️  Motor DERECHA - Velocidad: %d/255 (%.0f%%)",
             speed, (speed / 255.0) * 100);
}

void motor_stop(void)
{
    if (!initialized) {
        return;
    }

    // Detener: ENA=0, IN1=0, IN2=0
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_ENA_CHANNEL, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_ENA_CHANNEL);
    gpio_set_level(MOTOR_IN1_PIN, 0);
    gpio_set_level(MOTOR_IN2_PIN, 0);

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
