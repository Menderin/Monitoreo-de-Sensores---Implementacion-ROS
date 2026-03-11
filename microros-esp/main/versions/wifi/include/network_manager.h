/**
 * @file network_manager.h
 * @brief Gestión de la conexión WiFi — alta disponibilidad
 *
 * Provee:
 *  - Inicialización de la red (una sola vez, delegada a micro-ROS).
 *  - Verificación real de conectividad (IP válida vía esp_netif).
 *  - Tarea monitor que detecta caídas y reconecta usando ESP-IDF API.
 *  - Reinicio por hardware si el WiFi no se recupera en el tiempo límite.
 */

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <stdbool.h>

/**
 * @brief Inicializa la conexión WiFi a través de micro-ROS.
 *
 * Debe llamarse una única vez desde app_main antes de lanzar cualquier
 * tarea que dependa de la red.
 *
 * @return true  Si la IP fue asignada correctamente al retornar.
 * @return false Si la inicialización falló (el log detallará el motivo).
 */
bool network_manager_init(void);

/**
 * @brief Verifica si la interfaz WiFi tiene una IP válida asignada.
 *
 * Consulta esp_netif_get_ip_info() en lugar de un flag en memoria,
 * por lo que es segura para leer desde cualquier tarea/core.
 *
 * @return true  Si la IP es distinta de 0.0.0.0.
 * @return false Si no hay IP o la interfaz no está disponible.
 */
bool network_manager_is_connected(void);

/**
 * @brief Lanza la tarea FreeRTOS que monitorea la conexión WiFi.
 *
 * Debe llamarse una única vez, después de network_manager_init().
 * La tarea corre en el Core 0 con prioridad baja y:
 *  1. Verifica el estado cada WIFI_MONITOR_INTERVAL_MS.
 *  2. Si detecta pérdida, reintenta con esp_wifi_disconnect/connect.
 *  3. Si supera WIFI_MAX_RECONNECT_TIME_MS, llama a esp_restart().
 */
void network_manager_start_monitor(void);

/**
 * @brief Libera recursos de red y marca el estado como desconectado.
 *
 * Uso principalmente en secuencias de shutdown controlado.
 */
void network_manager_deinit(void);

#endif // NETWORK_MANAGER_H
