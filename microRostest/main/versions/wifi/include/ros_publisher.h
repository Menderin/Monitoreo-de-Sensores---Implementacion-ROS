/**
 * @file ros_publisher.h
 * @brief Abstracción de la capa de publicación micro-ROS
 * 
 * Gestiona la inicialización de micro-ROS y la publicación de datos
 */

#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include <stdbool.h>

/**
 * @brief Datos de los sensores para publicar
 */
typedef struct {
    float temperature;
    float ph;
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
 * @brief Limpia los recursos de micro-ROS
 */
void ros_publisher_deinit(void);

#endif // ROS_PUBLISHER_H
