/**
 * @file network_manager.c
 * @brief Implementación del gestor de red
 */

#include "../include/network_manager.h"
#include "../wifi_config.h"

#include "esp_log.h"
#include "esp_err.h"
#include <uros_network_interfaces.h>

static const char *TAG = "NETWORK_MANAGER";

static bool connected = false;

bool network_manager_init(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  INICIALIZANDO CONEXIÓN WiFi");
    ESP_LOGI(TAG, "========================================");
    
    // Usar la función del componente micro-ROS que gestiona WiFi automáticamente
    // Lee CONFIG_ESP_WIFI_SSID y CONFIG_ESP_WIFI_PASSWORD de sdkconfig
    esp_err_t ret = uros_network_interface_initialize();
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar WiFi: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  WiFi CONECTADO!");
    ESP_LOGI(TAG, "========================================");
    
    connected = true;
    return true;
}

bool network_manager_is_connected(void)
{
    return connected;
}

void network_manager_deinit(void)
{
    // La función uros_network_interface_initialize() no tiene un deinit explícito
    // pero podríamos agregar lógica de limpieza aquí si fuera necesario
    connected = false;
    ESP_LOGI(TAG, "Red desconectada");
}
