/**
 * @file motor_controller.c
 * @brief Controlador dual de motores DC — MOTOR_1 (GPIO25/26/14) y MOTOR_2 (GPIO27/33/32)
 */

#include "../include/motor_controller.h"
#include "../include/config.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

// ─────────────────────────────────────
// Tabla de hardware por motor
// ─────────────────────────────────────

typedef struct {
    gpio_num_t     in1_pin;
    gpio_num_t     in2_pin;
    gpio_num_t     en_pin;
    ledc_timer_t   timer;
    ledc_channel_t ch_in1;
    ledc_channel_t ch_in2;
    const char    *label;
} motor_hw_t;

static const motor_hw_t hw[MOTOR_COUNT] = {
    [MOTOR_1] = {
        .in1_pin = MOTOR_IN1_PIN,      // GPIO 25
        .in2_pin = MOTOR_IN2_PIN,      // GPIO 26
        .en_pin  = MOTOR_EN_PIN,       // GPIO 14
        .timer   = MOTOR_PWM_TIMER,    // LEDC_TIMER_0
        .ch_in1  = MOTOR_IN1_CHANNEL,  // LEDC_CHANNEL_0
        .ch_in2  = MOTOR_IN2_CHANNEL,  // LEDC_CHANNEL_1
        .label   = "MOTOR 1"
    },
    [MOTOR_2] = {
        .in1_pin = MOTOR2_IN1_PIN,     // GPIO 27
        .in2_pin = MOTOR2_IN2_PIN,     // GPIO 33
        .en_pin  = MOTOR2_EN_PIN,      // GPIO 32
        .timer   = MOTOR2_PWM_TIMER,   // LEDC_TIMER_1
        .ch_in1  = MOTOR2_IN1_CHANNEL, // LEDC_CHANNEL_2
        .ch_in2  = MOTOR2_IN2_CHANNEL, // LEDC_CHANNEL_3
        .label   = "MOTOR 2"
    },
};

// ─────────────────────────────────────
// Estado interno
// ─────────────────────────────────────

static bool       initialized   = false;
static uint8_t    current_speed = MOTOR_SPEED_DEFAULT;
static motor_id_t active_motor  = MOTOR_1;

// ─────────────────────────────────────
// Helpers privados
// ─────────────────────────────────────

static bool _init_motor(motor_id_t id)
{
    const motor_hw_t *m = &hw[id];

    // EN pin — salida digital HIGH
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << m->en_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&en_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "[%s] Error configurando pin EN", m->label);
        return false;
    }
    gpio_set_level(m->en_pin, 1);

    // Timer LEDC
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = MOTOR_PWM_MODE,
        .timer_num       = m->timer,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz         = MOTOR_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    if (ledc_timer_config(&timer_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "[%s] Error configurando timer PWM", m->label);
        return false;
    }

    // Canal IN1
    ledc_channel_config_t ch1 = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel    = m->ch_in1,
        .timer_sel  = m->timer,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = m->in1_pin,
        .duty       = 0,
        .hpoint     = 0
    };
    if (ledc_channel_config(&ch1) != ESP_OK) {
        ESP_LOGE(TAG, "[%s] Error configurando canal IN1", m->label);
        return false;
    }

    // Canal IN2
    ledc_channel_config_t ch2 = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel    = m->ch_in2,
        .timer_sel  = m->timer,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = m->in2_pin,
        .duty       = 0,
        .hpoint     = 0
    };
    if (ledc_channel_config(&ch2) != ESP_OK) {
        ESP_LOGE(TAG, "[%s] Error configurando canal IN2", m->label);
        return false;
    }

    ESP_LOGI(TAG, "  [%s]  EN=GPIO%-2d  IN1=GPIO%-2d  IN2=GPIO%-2d  %dHz",
             m->label, m->en_pin, m->in1_pin, m->in2_pin, MOTOR_PWM_FREQ);
    return true;
}

// ─────────────────────────────────────
// Inicialización pública
// ─────────────────────────────────────

bool motor_controller_init(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  INICIALIZANDO MOTORES (M1 + M2)");
    ESP_LOGI(TAG, "========================================");

    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!_init_motor((motor_id_t)i)) {
            return false;
        }
    }

    ESP_LOGI(TAG, "  Velocidad por defecto: %d/255 (%.0f%%)",
             current_speed, (current_speed / 255.0) * 100);
    ESP_LOGI(TAG, "  Motor activo: %s", hw[active_motor].label);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  MOTORES INICIALIZADOS OK");
    ESP_LOGI(TAG, "========================================");

    initialized = true;
    return true;
}

// ─────────────────────────────────────
// Selección de motor activo
// ─────────────────────────────────────

void motor_select(motor_id_t id)
{
    if (id >= MOTOR_COUNT) return;
    active_motor = id;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  >> Motor activo: %s <<", hw[id].label);
    ESP_LOGI(TAG, "  %s",
             (id == MOTOR_1)
             ? "[ M1 ] <-- activo     [ M2 ]"
             : "[ M1 ]     activo --> [ M2 ]");
    ESP_LOGI(TAG, "");
}

motor_id_t motor_get_selected(void)
{
    return active_motor;
}

// ─────────────────────────────────────
// Control por ID explícito
// ─────────────────────────────────────

void motor_move_left_id(motor_id_t id, uint8_t speed)
{
    if (!initialized || id >= MOTOR_COUNT) return;
    const motor_hw_t *m = &hw[id];

    ledc_set_duty(MOTOR_PWM_MODE, m->ch_in1, speed);
    ledc_update_duty(MOTOR_PWM_MODE, m->ch_in1);
    ledc_set_duty(MOTOR_PWM_MODE, m->ch_in2, 0);
    ledc_update_duty(MOTOR_PWM_MODE, m->ch_in2);

    ESP_LOGI(TAG, "[%s] <-- IZQ  %d/255 (%.0f%%)",
             m->label, speed, (speed / 255.0) * 100);
}

void motor_move_right_id(motor_id_t id, uint8_t speed)
{
    if (!initialized || id >= MOTOR_COUNT) return;
    const motor_hw_t *m = &hw[id];

    ledc_set_duty(MOTOR_PWM_MODE, m->ch_in1, 0);
    ledc_update_duty(MOTOR_PWM_MODE, m->ch_in1);
    ledc_set_duty(MOTOR_PWM_MODE, m->ch_in2, speed);
    ledc_update_duty(MOTOR_PWM_MODE, m->ch_in2);

    ESP_LOGI(TAG, "[%s] --> DER  %d/255 (%.0f%%)",
             m->label, speed, (speed / 255.0) * 100);
}

void motor_stop_id(motor_id_t id)
{
    if (!initialized || id >= MOTOR_COUNT) return;
    const motor_hw_t *m = &hw[id];

    ledc_set_duty(MOTOR_PWM_MODE, m->ch_in1, 0);
    ledc_update_duty(MOTOR_PWM_MODE, m->ch_in1);
    ledc_set_duty(MOTOR_PWM_MODE, m->ch_in2, 0);
    ledc_update_duty(MOTOR_PWM_MODE, m->ch_in2);

    ESP_LOGI(TAG, "[%s] || DETENIDO", m->label);
}

// ─────────────────────────────────────
// Control sobre el motor activo
// ─────────────────────────────────────

void motor_move_left(uint8_t speed)  { motor_move_left_id(active_motor, speed); }
void motor_move_right(uint8_t speed) { motor_move_right_id(active_motor, speed); }
void motor_stop(void)                { motor_stop_id(active_motor); }

void motor_set_speed(uint8_t speed)
{
    current_speed = speed;
    ESP_LOGI(TAG, "  Velocidad: %d/255 (%.0f%%)", speed, (speed / 255.0) * 100);
}

uint8_t motor_get_speed(void) { return current_speed; }
