/**
 * @file motor_controller.h
 * @brief Controlador de motor DC con driver MINI 298 y PWM directo
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Inicializa el controlador de motor con PWM
 * 
 * @return true si la inicialización fue exitosa, false en caso contrario
 */
bool motor_controller_init(void);

/**
 * @brief Mueve el motor hacia la izquierda con velocidad especificada
 * 
 * @param speed Velocidad (0-255): MOTOR_SPEED_SLOW, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_FAST
 */
void motor_move_left(uint8_t speed);

/**
 * @brief Mueve el motor hacia la derecha convelocidad especificada
 * 
 * @param speed Velocidad (0-255): MOTOR_SPEED_SLOW, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_FAST
 */
void motor_move_right(uint8_t speed);

/**
 * @brief Detiene el motor
 */
void motor_stop(void);

/**
 * @brief Establece la velocidad global del motor (para comandos sin parámetro)
 * 
 * @param speed Velocidad (0-255)
 */
void motor_set_speed(uint8_t speed);

/**
 * @brief Obtiene la velocidad actual configurada
 * 
 * @return Velocidad actual (0-255)
 */
uint8_t motor_get_speed(void);

#endif // MOTOR_CONTROLLER_H
