/**
 * @file motor_controller.h
 * @brief Controlador dual de motores DC con driver MINI 298 y PWM directo
 *
 * Motor 1 (cableado listo):
 *   IN1=GPIO25  IN2=GPIO26  EN=GPIO14
 *
 * Motor 2:
 *   IN1=GPIO27  IN2=GPIO33  EN=GPIO32
 *
 * Uso básico:
 *   motor_controller_init();          // inicializa ambos motores
 *   motor_select(MOTOR_1);            // selecciona motor activo
 *   motor_move_left(current_speed);   // mueve el motor activo
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

// ─────────────────────────────────────
// Identificador de motor
// ─────────────────────────────────────

typedef enum {
    MOTOR_1 = 0,   ///< GPIO 25/26/14
    MOTOR_2 = 1,   ///< GPIO 27/33/32
    MOTOR_COUNT
} motor_id_t;

// ─────────────────────────────────────
// Inicialización
// ─────────────────────────────────────

/**
 * @brief Inicializa ambos motores (PWM + EN pin en HIGH).
 * @return true si OK.
 */
bool motor_controller_init(void);

// ─────────────────────────────────────
// Selección de motor activo
// ─────────────────────────────────────

/**
 * @brief Selecciona el motor sobre el que actuarán los comandos simples.
 * @param id MOTOR_1 o MOTOR_2
 */
void motor_select(motor_id_t id);

/**
 * @brief Retorna el motor actualmente seleccionado.
 */
motor_id_t motor_get_selected(void);

// ─────────────────────────────────────
// Control (actúan sobre el motor activo)
// ─────────────────────────────────────

void motor_move_left(uint8_t speed);
void motor_move_right(uint8_t speed);
void motor_stop(void);
void motor_set_speed(uint8_t speed);
uint8_t motor_get_speed(void);

// ─────────────────────────────────────
// Control explícito por ID
// ─────────────────────────────────────

void motor_move_left_id(motor_id_t id, uint8_t speed);
void motor_move_right_id(motor_id_t id, uint8_t speed);
void motor_stop_id(motor_id_t id);

#endif // MOTOR_CONTROLLER_H
