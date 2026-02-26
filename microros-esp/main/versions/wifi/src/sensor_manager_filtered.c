/**
 * @file sensor_manager_filtered.c
 * @brief Implementación del gestor de sensores CON FILTRADO DE RUIDO
 * 
 * Versión mejorada que incluye:
 * - MEDIANA de 10 muestras por lectura (más robusto que promedio)
 * - Eliminación automática de outliers y spikes
 * - Reducción de ruido del ADC del ESP32
 * - Lecturas más estables y confiables
 * 
 * Cambios respecto a sensor_manager.c:
 * - sensor_read_temperature(): Mediana de 10 lecturas
 * - sensor_read_ph(): Mediana de 10 lecturas
 */

#include "../include/sensor_manager.h"
#include "../include/config.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SENSOR_MANAGER";

// Configuración del filtro de promediado
#define FILTER_NUM_SAMPLES  10   // Número de muestras a promediar
#define FILTER_DELAY_MS     2    // Delay entre muestras (ms)

// Handles del ADC
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// ========================================
// FUNCIONES PRIVADAS DE CONVERSIÓN
// ========================================

/**
 * @brief Función de comparación para qsort
 */
static int compare_int(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

/**
 * @brief Calcula la mediana de un array de enteros
 * @param arr Array de valores
 * @param len Longitud del array
 * @return Valor de la mediana
 */
static int calculate_median(int *arr, int len) {
    if (len == 0) return 0;
    
    // Ordenar array (usa quicksort de la stdlib)
    qsort(arr, len, sizeof(int), compare_int);
    
    // Calcular mediana
    if (len % 2 == 0) {
        // Par: promedio de los dos valores centrales
        return (arr[len/2 - 1] + arr[len/2]) / 2;
    } else {
        // Impar: valor central
        return arr[len/2];
    }
}

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
    ESP_LOGI(TAG, "Inicializando sistema de sensores (CON FILTRADO)...");
    
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
    ESP_LOGI(TAG, "Filtro de ruido: MEDIANA de %d muestras", FILTER_NUM_SAMPLES);
    ESP_LOGI(TAG, "Sistema de sensores inicializado (VERSIÓN CON MEDIANA)");
    
    return true;
}

float sensor_read_temperature(void)
{
    int raw, voltage_mv;
    int samples[FILTER_NUM_SAMPLES];  // Array para almacenar muestras
    int num_samples = 0;
    
    // ========================================
    // FILTRADO: Recolectar múltiples muestras
    // ========================================
    for (int i = 0; i < FILTER_NUM_SAMPLES; i++) {
        esp_err_t ret = adc_oneshot_read(adc_handle, ADC_TEMP_CHANNEL, &raw);
        
        if (ret == ESP_OK) {
            samples[num_samples] = raw;
            num_samples++;
        } else {
            ESP_LOGW(TAG, "Falló lectura %d de temperatura", i+1);
        }
        
        // Pequeña pausa entre lecturas para estabilidad
        if (i < FILTER_NUM_SAMPLES - 1) {  // No delay en última iteración
            vTaskDelay(pdMS_TO_TICKS(FILTER_DELAY_MS));
        }
    }
    
    // Verificar que tengamos al menos algunas lecturas exitosas
    if (num_samples == 0) {
        ESP_LOGE(TAG, "Error: No se pudo leer temperatura");
        return 0.0;
    }
    
    // Calcular MEDIANA de las muestras (más robusto que promedio)
    raw = calculate_median(samples, num_samples);
    
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
    int samples[FILTER_NUM_SAMPLES];  // Array para almacenar muestras
    int num_samples = 0;
    
    // ========================================
    // FILTRADO: Recolectar múltiples muestras
    // ========================================
    for (int i = 0; i < FILTER_NUM_SAMPLES; i++) {
        esp_err_t ret = adc_oneshot_read(adc_handle, ADC_PH_CHANNEL, &raw);
        
        if (ret == ESP_OK) {
            samples[num_samples] = raw;
            num_samples++;
        } else {
            ESP_LOGW(TAG, "Falló lectura %d de pH", i+1);
        }
        
        // Pequeña pausa entre lecturas para estabilidad
        if (i < FILTER_NUM_SAMPLES - 1) {  // No delay en última iteración
            vTaskDelay(pdMS_TO_TICKS(FILTER_DELAY_MS));
        }
    }
    
    // Verificar que tengamos al menos algunas lecturas exitosas
    if (num_samples == 0) {
        ESP_LOGE(TAG, "Error: No se pudo leer pH");
        return 7.0; // Valor neutro por defecto
    }
    
    // Calcular MEDIANA de las muestras (más robusto que promedio)
    raw = calculate_median(samples, num_samples);
    
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
    int samples[FILTER_NUM_SAMPLES];  // Array para almacenar muestras
    int num_samples = 0;
    
    // ========================================
    // FILTRADO: Recolectar múltiples muestras
    // ========================================
    for (int i = 0; i < FILTER_NUM_SAMPLES; i++) {
        esp_err_t ret = adc_oneshot_read(adc_handle, ADC_PH_CHANNEL, &raw);
        
        if (ret == ESP_OK) {
            samples[num_samples] = raw;
            num_samples++;
        }
        
        // Pequeña pausa entre lecturas
        if (i < FILTER_NUM_SAMPLES - 1) {
            vTaskDelay(pdMS_TO_TICKS(FILTER_DELAY_MS));
        }
    }
    
    if (num_samples == 0) {
        ESP_LOGE(TAG, "Error al leer pH raw");
        return 0.0;
    }
    
    // Calcular MEDIANA de las muestras
    raw = calculate_median(samples, num_samples);
    
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
