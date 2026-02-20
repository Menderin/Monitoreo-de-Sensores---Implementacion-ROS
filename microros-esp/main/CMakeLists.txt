# Generar wifi_config.h desde .env antes de compilar
# Usar CMAKE_CURRENT_LIST_DIR que siempre apunta al directorio donde está este CMakeLists.txt
set(MAIN_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(ENV_FILE "${MAIN_SRC_DIR}/versions/wifi/.env")
set(WIFI_CONFIG_H "${MAIN_SRC_DIR}/versions/wifi/wifi_config.h")

# Verificar que existe el archivo .env
if(NOT EXISTS "${ENV_FILE}")
    message(FATAL_ERROR "❌ No se encontró el archivo .env en: ${ENV_FILE}")
endif()

# Leer el archivo .env
file(READ "${ENV_FILE}" ENV_CONTENTS)

# Parsear variables del .env
string(REGEX MATCH "WIFI_SSID=([^\n]*)" _ "${ENV_CONTENTS}")
set(WIFI_SSID "${CMAKE_MATCH_1}")

string(REGEX MATCH "WIFI_PASSWORD=([^\n]*)" _ "${ENV_CONTENTS}")
set(WIFI_PASSWORD "${CMAKE_MATCH_1}")

string(REGEX MATCH "AGENT_IP=([^\n]*)" _ "${ENV_CONTENTS}")
set(AGENT_IP "${CMAKE_MATCH_1}")

string(REGEX MATCH "AGENT_PORT=([^\n]*)" _ "${ENV_CONTENTS}")
set(AGENT_PORT "${CMAKE_MATCH_1}")

# Validar que las variables no estén vacías
if("${WIFI_SSID}" STREQUAL "" OR "${WIFI_PASSWORD}" STREQUAL "" OR "${AGENT_IP}" STREQUAL "" OR "${AGENT_PORT}" STREQUAL "")
    message(FATAL_ERROR "❌ Error: Faltan variables en .env. Verifica WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT")
endif()

# Generar wifi_config.h
file(WRITE "${WIFI_CONFIG_H}" 
"/**
 * @file wifi_config.h
 * @brief Configuración WiFi y micro-ROS Agent
 * 
 * GENERADO AUTOMÁTICAMENTE desde .env
 * NO EDITAR MANUALMENTE - Los cambios se perderán
 */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// Configuración WiFi
#define WIFI_SSID           \"${WIFI_SSID}\"
#define WIFI_PASSWORD       \"${WIFI_PASSWORD}\"
#define WIFI_MAX_RETRY      10

// Configuración micro-ROS Agent (UDP)
#define AGENT_IP            \"${AGENT_IP}\"
#define AGENT_PORT          ${AGENT_PORT}
#define AGENT_PORT_STR      \"${AGENT_PORT}\"

#endif // WIFI_CONFIG_H
")

message(STATUS "✅ Generado wifi_config.h - SSID: ${WIFI_SSID}, Agent: ${AGENT_IP}:${AGENT_PORT}")

# ============================================================================
# SELECCIÓN DE VERSIÓN
# Comenta la versión que NO quieres usar
# ============================================================================

# OPCIÓN A: Versión WiFi/UDP (modular) - Para Agent UDP
idf_component_register(
    SRCS 
        "versions/wifi/src/main.c"
        "versions/wifi/src/sensor_manager_filtered.c"
        "versions/wifi/src/ros_publisher.c"
        "versions/wifi/src/network_manager.c"
        "versions/wifi/src/motor_controller.c"
    INCLUDE_DIRS 
        "." 
        "versions/wifi"
        "versions/wifi/include"
    REQUIRES 
        micro_ros_espidf_component 
        esp_adc 
        esp_wifi 
        esp_netif 
        nvs_flash
        driver
)

# ============================================================================
# VERIFICACIÓN DE VERSIÓN DEL SENSOR MANAGER
# ============================================================================
message(STATUS "")
message(STATUS "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
message(STATUS "📊 VERSIÓN DE SENSOR MANAGER: FILTRADA (MEDIANA)")
message(STATUS "   Archivo: sensor_manager_filtered.c")
message(STATUS "   Filtro: Mediana de 10 muestras")
message(STATUS "   Beneficios: Elimina outliers y spikes")
message(STATUS "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
message(STATUS "")

# OPCIÓN B: Versión Serial/UART - Para Agent Serial
# Descomenta esta y comenta la OPCIÓN A para usar serial
# idf_component_register(
#     SRCS "versions/sensor_temp_ph/sensor_tempV3.c"
#          "esp32_serial_transport.c"
#     INCLUDE_DIRS "." "versions/sensor_temp_ph"
#     REQUIRES micro_ros_espidf_component esp_adc driver
# )

# UDP es el transporte por defecto, no necesita macro especial