/**
 * @file sensor_manager.h
 * @brief Gestión de sensores CWT-BL (temperatura y pH)
 * 
 * Abstrae toda la lógica de ADC, calibración y conversión de sensores
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdbool.h>

/**
 * @brief Inicializa el ADC y configura los canales de los sensores
 * @return true si la inicialización fue exitosa, false en caso contrario
 */
bool sensor_manager_init(void);

/**
 * @brief Lee el sensor de temperatura
 * @return Temperatura en grados Celsius
 */
float sensor_read_temperature(void);

/**
 * @brief Lee el sensor de pH
 * @return Valor de pH (0.0 - 14.0)
 */
float sensor_read_ph(void);

/**
 * @brief Lee el voltaje raw del sensor de pH (para calibración)
 * @return Voltaje en mV
 */
float sensor_read_ph_voltage_raw(void);

/**
 * @brief Limpia los recursos del ADC
 */
void sensor_manager_deinit(void);

#endif // SENSOR_MANAGER_H
