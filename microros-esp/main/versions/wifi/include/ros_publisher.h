/**
 * @file ros_publisher.h
 * @brief Abstracción de la capa de publicación micro-ROS
 * 
 * Gestiona la inicialización de micro-ROS y la publicación de datos
 */

#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Datos de los sensores para publicar
 */
typedef struct {
    float temperature;
    float ph;
    float voltage_raw_ph;  // Voltaje raw en mV para calibración
} sensor_data_t;

/**
 * @brief Inicializa micro-ROS (soporte, nodo y publicadores)
 * @return true si la inicialización fue exitosa, false en caso contrario
 */
bool ros_publisher_init(void);

/**
 * @brief Publica los datos de los sensores
 * @param data Datos de temperatura y pH a publicar
 * @return true si la publicación fue exitosa, false en caso contrario
 */
bool ros_publisher_publish(const sensor_data_t *data);

/**
 * @brief Ejecuta el executor para procesar callbacks de subscriptores
 * @param timeout_ns Timeout en nanosegundos (usar RCL_MS_TO_NS(ms) para convertir)
 * @return true si se procesó exitosamente, false en caso contrario
 */
bool ros_executor_spin_some(uint64_t timeout_ns);

/**
 * @brief Limpia los recursos de micro-ROS
 */
void ros_publisher_deinit(void);

/**
 * @brief Verifica si el Agente ROS es alcanzable; ejecuta hot-reload si la
 *        sesión está perdida. Debe llamarse periódicamente desde el loop
 *        principal (recomendado: cada ROS_AGENT_PING_INTERVAL_MS ms).
 *
 * Flujo interno:
 *  1. Ping al Agente con rmw_uros_ping_agent_options() (incluye IP/puerto UDP).
 *  2. Hasta ROS_AGENT_MAX_RETRIES reintentos con pausa entre cada uno.
 *  3. Si todos fallan: destroy → pausa de seguridad → create (hot-reload).
 *
 * @return true  Si el sistema ROS está operativo (ping OK o hot-reload exitoso).
 * @return false Si el hot-reload también falló; se reintentará en el próximo ciclo.
 */
bool ros_agent_check_and_reconnect(void);

#endif // ROS_PUBLISHER_H
