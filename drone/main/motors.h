/*
 *****************************************************************************
 * File: motors.h
 * Author: Jack Cairns
 * Date: 13-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

#include "driver/mcpwm_prelude.h"

// Motor/ESC's GPIO Pins
// Ensure these are sequential as the PWM setup requires this.
// If not possible alter the esc_pwm_init() function.
#define MOTOR_A_CW 4   // Front left
#define MOTOR_B_CCW 16 // Rear left
#define MOTOR_C_CW 17  // Rear right
#define MOTOR_D_CCW 5  // Front right

// Throttle speeds / Duty cycle's in us
#define MOTOR_SPEED_MIN 1000
#define MOTOR_SPEED_MAX 2000
#define MAX_MOTOR_ADJUSTMENT 150
#define NUMBER_OF_MOTORS 4

typedef enum { MOTOR_A, MOTOR_B, MOROT_C, MOTOR_D } MotorIndex;

/**
 * @brief Initializes the MCPWM hardware to drive four ESCs with PWM signals.
 *
 * This function configures:
 * - Two MCPWM timers (one per operator group), each running at 50 Hz (20 ms period).
 * - Four MCPWM operators (two per group), each attached to a timer.
 * - Four comparators (one per ESC) to control when the PWM signal goes LOW.
 * - Four generators mapped to GPIOs to output the PWM waveform to each ESC.
 *
 * The PWM waveform goes HIGH at the beginning of each timer cycle and LOW when
 * the comparator value is reached. This simulates RC-style PWM used by most ESCs.
 *
 * @note Must be called once during startup before using esc_pwm_set_duty_cycle().
 * @note Assumes MOTOR_* constants define valid GPIO pins.
 *
 * @global esc_pwm_comparators[] Will be populated with comparator handles.
 *
 * @return ESP_OK on success, or error code on failure
 */
esp_err_t esc_pwm_init(void);

/**
 * @brief Updates the duty cycle (pulse width) of the selected motor's ESC.
 *
 * Sets the comparator value for the specified ESC's MCPWM generator,
 * controlling when the PWM signal transitions from HIGH to LOW.
 *
 * The input duty cycle is clamped between MOTOR_SPEED_MIN and MOTOR_SPEED_MAX.
 *
 * @param motor      Index of the motor (0 to NUMBER_OF_MOTORS - 1).
 * @param duty_cycle Desired pulse width in microseconds (typically 1000–2000 µs).
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for invalid motor index,
 *         or other ESP error codes from MCPWM operations.
 *
 * @note Requires esc_pwm_init() to be called first.
 */
esp_err_t esc_pwm_set_duty_cycle(MotorIndex motor, uint16_t duty_cycle);

#endif
