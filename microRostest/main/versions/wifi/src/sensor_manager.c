/**
 * @file sensor_manager.c
 * @brief Implementación del gestor de sensores
 */

#include "../include/sensor_manager.h"
#include "../include/config.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "SENSOR_MANAGER";

// Handles del ADC
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// ========================================
// FUNCIONES PRIVADAS DE CONVERSIÓN
// ========================================

static float voltage_to_temperature(int voltage_mv) {
    // Ajuste de voltaje para rango de 0-5V (el ESP32 lee 0-3.3V)
    float voltage_real_v = (voltage_mv / 1000.0) * (5.0 / 3.3);
    // Conversión según especificaciones del sensor
    float temp = voltage_real_v * 20.0 - 20.0 + TEMP_OFFSET_CAL;
    return temp;
}

static float voltage_to_ph(int voltage_mv) {
    // Conversión lineal de mV a pH
    float ph = (voltage_mv * PH_SLOPE) + PH_INTERCEPT;
    // Limitar al rango válido de pH
    if (ph < 0.0) ph = 0.0;
    if (ph > 14.0) ph = 14.0;
    return ph;
}

// ========================================
// FUNCIONES PÚBLICAS
// ========================================

bool sensor_manager_init(void)
{
    ESP_LOGI(TAG, "Inicializando sistema de sensores...");
    
    // Configurar unidad ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar ADC: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configurar canales
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN,
    };
    
    ret = adc_oneshot_config_channel(adc_handle, ADC_TEMP_CHANNEL, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar canal de temperatura: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = adc_oneshot_config_channel(adc_handle, ADC_PH_CHANNEL, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar canal de pH: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Calibración del ADC
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle) == ESP_OK) {
        ESP_LOGI(TAG, "Calibración ADC: OK");
    } else {
        ESP_LOGW(TAG, "Calibración ADC no disponible, usando valores RAW");
    }
    
    ESP_LOGI(TAG, "Canal temperatura: GPIO39 (ADC1_CH3)");
    ESP_LOGI(TAG, "Canal pH: GPIO36 (ADC1_CH0)");
    ESP_LOGI(TAG, "Sistema de sensores inicializado");
    
    return true;
}

float sensor_read_temperature(void)
{
    int raw, voltage_mv;
    
    // Leer valor raw del ADC
    esp_err_t ret = adc_oneshot_read(adc_handle, ADC_TEMP_CHANNEL, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer temperatura: %s", esp_err_to_name(ret));
        return 0.0;
    }
    
    // Convertir a voltaje
    if (adc_cali_handle) {
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv);
    } else {
        // Conversión manual si no hay calibración
        voltage_mv = raw * 3300 / 4095;
    }
    
    return voltage_to_temperature(voltage_mv);
}

float sensor_read_ph(void)
{
    int raw, voltage_mv;
    
    // Leer valor raw del ADC
    esp_err_t ret = adc_oneshot_read(adc_handle, ADC_PH_CHANNEL, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer pH: %s", esp_err_to_name(ret));
        return 7.0; // Valor neutro por defecto
    }
    
    // Convertir a voltaje
    if (adc_cali_handle) {
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv);
    } else {
        // Conversión manual si no hay calibración
        voltage_mv = raw * 3300 / 4095;
    }
    
    return voltage_to_ph(voltage_mv);
}

float sensor_read_ph_voltage_raw(void)
{
    int raw, voltage_mv;
    
    // Leer valor raw del ADC
    esp_err_t ret = adc_oneshot_read(adc_handle, ADC_PH_CHANNEL, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer pH raw: %s", esp_err_to_name(ret));
        return 0.0;
    }
    
    // Convertir a voltaje
    if (adc_cali_handle) {
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_mv);
    } else {
        // Conversión manual si no hay calibración
        voltage_mv = raw * 3300 / 4095;
    }
    
    return (float)voltage_mv;
}

void sensor_manager_deinit(void)
{
    if (adc_cali_handle) {
        adc_cali_delete_scheme_line_fitting(adc_cali_handle);
        adc_cali_handle = NULL;
    }
    
    if (adc_handle) {
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Sistema de sensores detenido");
}
