/**
 * @file network_manager.c
 * @brief Gestor de red WiFi de alta disponibilidad
 *
 * Responsabilidades de este módulo (SRP):
 *  - Inicialización del stack de red (una vez, vía micro-ROS helper).
 *  - Verificación real de IP usando esp_netif.
 *  - Tarea FreeRTOS que monitorea la conexión de forma periódica.
 *  - Reconexión con la API nativa ESP-IDF (sin re-llamar a micro-ROS).
 *  - Reinicio de hardware tras superar el tiempo límite de reconexión.
 *
 * Lo que NO hace este módulo:
 *  - Leer sensores.
 *  - Publicar datos ROS.
 *  - Gestionar credenciales WiFi (eso lo hace wifi_config.h + sdkconfig).
 *
 * Thread-safety:
 *  - El estado de conexión se determina en tiempo real consultando la IP
 *    (network_manager_is_connected), por lo que no necesita mutex propio.
 *  - La tarea monitor corre en Core 0; la tarea micro-ROS en Core 1.
 *    Ambas pueden llamar is_connected() de forma concurrente sin riesgos
 *    porque esp_netif_get_ip_info() es reentrante.
 *
 * @author ESP32 micro-ROS Team
 * @date 2026
 */

#include "../include/network_manager.h"
#include "../include/config.h"

/* ESP-IDF: red */
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_system.h"         /* esp_restart() */

/* micro-ROS helper (solo para la inicialización inicial) */
#include <uros_network_interfaces.h>

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ------------------------------------------------------------------ */
/* Privado                                                             */
/* ------------------------------------------------------------------ */

static const char *TAG = "NETWORK_MANAGER";

/**
 * @brief Obtiene el handle de la interfaz de estación WiFi.
 *
 * esp_netif_get_handle_from_ifkey() devuelve NULL si el stack de red
 * no fue inicializado, por lo que sirve como comprobación implícita.
 *
 * @return Puntero a la interfaz, o NULL si no existe.
 */
static esp_netif_t *get_sta_netif(void)
{
    return esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
}

/**
 * @brief Intenta una única reconexión WiFi usando la API ESP-IDF.
 *
 * Secuencia:
 *  1. esp_wifi_disconnect()  — asegura que el driver esté en estado limpio.
 *  2. vTaskDelay breve       — permite que el event loop procese el evento.
 *  3. esp_wifi_connect()     — inicia la asociación al AP.
 *  4. Polling de IP          — espera hasta WIFI_IP_WAIT_MS a que DHCP
 *                              asigne una dirección válida.
 *
 * No bloquea indefinidamente: el bucle de polling siempre tiene un
 * vTaskDelay, por lo que el Watchdog del SO no se dispara.
 *
 * @return true  Si se obtuvo una IP válida dentro del tiempo límite.
 * @return false Si expiró el tiempo o la llamada a esp_wifi_connect falló.
 */
static bool wifi_reconnect_once(void)
{
    ESP_LOGW(TAG, "Iniciando secuencia de reconexión...");

    /* 1. Desconectar limpiamente (ignoramos el error: podría ya estar
     *    desconectado y eso es perfectamente válido). */
    esp_wifi_disconnect();

    /* 2. Pausa breve para que el driver procese la desconexión. */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 3. Intentar conectar al AP configurado en sdkconfig / wifi_config.h */
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect() falló: %s", esp_err_to_name(err));
        return false;
    }

    /* 4. Esperar a que DHCP asigne una IP válida (polling no bloqueante). */
    const uint32_t max_polls = WIFI_IP_WAIT_MS / WIFI_IP_POLL_MS;
    esp_netif_t *sta = get_sta_netif();

    for (uint32_t i = 0; i < max_polls; i++) {
        vTaskDelay(pdMS_TO_TICKS(WIFI_IP_POLL_MS));   /* WDT-safe: siempre cede */

        if (sta == NULL) {
            /* La interfaz aún no existe; reintentamos en la siguiente iteración. */
            sta = get_sta_netif();
            continue;
        }

        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(sta, &ip_info) == ESP_OK &&
            ip_info.ip.addr != 0)
        {
            ESP_LOGI(TAG, "Reconexión exitosa. IP: " IPSTR,
                     IP2STR(&ip_info.ip));
            return true;
        }
    }

    ESP_LOGW(TAG, "Tiempo de espera de IP agotado tras reconexión.");
    return false;
}

/**
 * @brief Tarea FreeRTOS: monitor activo de la conexión WiFi.
 *
 * Corre en Core 0 con prioridad baja (WIFI_MONITOR_PRIORITY).
 * No contiene ningún bucle bloqueante: todos los while/for internos
 * tienen vTaskDelay, por lo que FreeRTOS puede planificar otras tareas
 * y el Hardware Watchdog Timer nunca se dispara por este módulo.
 *
 * Lógica de estados:
 *
 *   [CONECTADO] ──── pérdida detectada ───▶ [RECONECTANDO]
 *       ▲                                        │
 *       └──────────── éxito ────────────────────┘
 *                                                │ supera WIFI_MAX_RECONNECT_TIME_MS
 *                                                ▼
 *                                         [esp_restart()]
 *
 * @param arg No utilizado.
 */
static void wifi_monitor_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Monitor WiFi iniciado (intervalo: %u ms, timeout: %u s).",
             (unsigned)WIFI_MONITOR_INTERVAL_MS,
             (unsigned)(WIFI_MAX_RECONNECT_TIME_MS / 1000U));

    /* Acumulador de tiempo desconectado (en ms). */
    uint32_t disconnected_ms = 0;

    while (1) {
        /* ── Pausa periódica: cede al SO entre cada ciclo de chequeo ── */
        vTaskDelay(pdMS_TO_TICKS(WIFI_MONITOR_INTERVAL_MS));

        if (network_manager_is_connected()) {
            /* Estado saludable: resetear el acumulador. */
            if (disconnected_ms > 0) {
                ESP_LOGI(TAG, "Conexión WiFi restaurada.");
                disconnected_ms = 0;
            }
            continue;
        }

        /* ── Pérdida de conexión detectada ── */
        disconnected_ms += WIFI_MONITOR_INTERVAL_MS;

        ESP_LOGW(TAG,
                 "WiFi sin IP. Tiempo desconectado: %lu s / %lu s máx.",
                 (unsigned long)(disconnected_ms / 1000U),
                 (unsigned long)(WIFI_MAX_RECONNECT_TIME_MS / 1000U));

        /* ¿Superamos el límite de tiempo sin WiFi? */
        if (disconnected_ms >= WIFI_MAX_RECONNECT_TIME_MS) {
            ESP_LOGE(TAG,
                     "Timeout de reconexión superado (%lu s). "
                     "Reiniciando el hardware para limpiar el estado del driver.",
                     (unsigned long)(WIFI_MAX_RECONNECT_TIME_MS / 1000U));
            /* Pequeña pausa para que el log se vuelque al buffer de serie. */
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
            /* (nunca se alcanza) */
        }

        /* ── Intento de reconexión ── */
        ESP_LOGI(TAG, "Intentando reconectar al WiFi...");
        if (wifi_reconnect_once()) {
            disconnected_ms = 0;   /* Éxito: resetear el acumulador. */
        } else {
            /* Fallo: esperar antes del siguiente intento.
             * WIFI_RETRY_DELAY_MS ya forma parte de WIFI_MONITOR_INTERVAL_MS
             * del siguiente ciclo, pero agregamos una pausa adicional aquí
             * para no saturar el driver con reintentos demasiado rápidos. */
            ESP_LOGW(TAG, "Reconexión fallida. Próximo intento en %u s.",
                     (unsigned)(WIFI_RETRY_DELAY_MS / 1000U));
            vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY_MS));
            disconnected_ms += WIFI_RETRY_DELAY_MS;
        }
    }
    /* No se alcanza, pero buena práctica en FreeRTOS: */
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* API pública                                                         */
/* ------------------------------------------------------------------ */

bool network_manager_is_connected(void)
{
    esp_netif_t *sta = get_sta_netif();
    if (sta == NULL) {
        return false;
    }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(sta, &ip_info) != ESP_OK) {
        return false;
    }

    return (ip_info.ip.addr != 0);
}

bool network_manager_init(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  INICIALIZANDO CONEXIÓN WiFi");
    ESP_LOGI(TAG, "========================================");

    /*
     * uros_network_interface_initialize() gestiona todo el stack WiFi
     * del ESP-IDF (nvs_flash, netif, event loop, driver) y bloquea
     * hasta obtener IP. Se llama UNA SOLA VEZ aquí; las reconexiones
     * posteriores usan la API nativa esp_wifi_connect().
     */
    esp_err_t ret = uros_network_interface_initialize();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar WiFi: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  WiFi CONECTADO — IP asignada");
    ESP_LOGI(TAG, "========================================");

    return true;
}

void network_manager_start_monitor(void)
{
    BaseType_t result = xTaskCreatePinnedToCore(
        wifi_monitor_task,          /* Función de la tarea              */
        "wifi_monitor",             /* Nombre (visible en idf.py monitor)*/
        WIFI_MONITOR_STACK_SIZE,    /* Stack (bytes)                    */
        NULL,                       /* Parámetro                        */
        WIFI_MONITOR_PRIORITY,      /* Prioridad (baja, no compite)     */
        NULL,                       /* Handle (no necesitamos guardarlo)*/
        WIFI_MONITOR_CORE           /* Core 0 / PRO_CPU                 */
    );

    if (result != pdPASS) {
        /*
         * Si no hay RAM suficiente para crear la tarea, registramos el
         * error pero NO hacemos abort: el sistema puede continuar
         * operando; solo pierde la capacidad de reconexión automática.
         */
        ESP_LOGE(TAG,
                 "No se pudo crear la tarea wifi_monitor (sin RAM). "
                 "El sistema operará sin reconexión automática.");
    } else {
        ESP_LOGI(TAG, "Tarea wifi_monitor iniciada en Core %d, prioridad %d.",
                 WIFI_MONITOR_CORE, WIFI_MONITOR_PRIORITY);
    }
}

void network_manager_deinit(void)
{
    /* Detener driver WiFi de forma limpia. */
    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_disconnect() retornó: %s",
                 esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "Red desconectada — recursos liberados.");
}
