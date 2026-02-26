/**
 * @file network_manager.h
 * @brief Gestión de la conexión WiFi
 * 
 * Abstrae la inicialización y gestión de la red
 */

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <stdbool.h>

/**
 * @brief Inicializa la conexión WiFi
 * @return true si la conexión fue exitosa, false en caso contrario
 */
bool network_manager_init(void);

/**
 * @brief Verifica si la WiFi está conectada
 * @return true si está conectada, false en caso contrario
 */
bool network_manager_is_connected(void);

/**
 * @brief Desconecta y limpia los recursos de red
 */
void network_manager_deinit(void);

#endif // NETWORK_MANAGER_H
