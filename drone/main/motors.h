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


#define MOTOR_A_OFFSET 125
#define MOTOR_B_OFFSET 65
#define MOTOR_C_OFFSET 0
#define MOTOR_D_OFFSET 185

// #define MOTOR_D_OFFSET // 65 is when it turns on
// #define MOTOR_A_OFFSET // 125 is when it turns on
// #define MOTOR_B_OFFSET // 185 is when it turns on
// #define MOTOR_C_OFFSET // 250 is when it turns on


typedef enum { MOTOR_A, MOTOR_B, MOROT_C, MOTOR_D } MotorIndex;

void esc_pwm_init(void);
void esc_pwm_set_duty_cycle(MotorIndex motor, uint16_t duty_cycle);

#endif
