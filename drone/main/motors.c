/*
 *****************************************************************************
 * File: motor.c
 * Author: Jack Cairns
 * Date: 13-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "motors.h"

mcpwm_cmpr_handle_t esc_pwm_comparators[4] = {NULL};

/* esc_pwm_init()
 * ---------------
 * Function to initialize the MCPWM peripheral for driving four ESC signals.
 *
 * This sets up:
 *   - One MCPWM timer to generate a 50 Hz PWM base (20 ms period) with 1 us resolution.
 *   - Two MCPWM operators connected to the timer.
 *   - Four MCPWM comparators (one per ESC signal) distributed across the operators.
 *   - Four MCPWM generators attached to GPIO pins, each outputting a PWM waveform.
 *   - Each generator is configured to set its output HIGH at the start of the period
 *     and LOW when its comparator value is reached.
 *
 * The function must be called once during system initialization before using
 * esc_pwm_set_duty_cycle() to control the throttle of each ESC.
 *
 * Globals:
 *   esc_pwm_comparators[] - The array of comparator handles used by esc_pwm_set_duty_cycle().
 */
void esc_pwm_init(void) {

    // Create and configure the timer
    mcpwm_timer_handle_t pwm_timers[2] = {NULL};
    for (uint8_t i = 0; i < 2; i++) {
        mcpwm_timer_config_t pwm_timer_config = {
            .resolution_hz = 1000000,                // 1MHz
            .period_ticks = 20000,                   // 20ms
            .group_id = i,                           // Group 0
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP, // Count up from 0 then reset to 0
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT   // Default clock source
        };

        ESP_ERROR_CHECK(mcpwm_new_timer(&pwm_timer_config, &pwm_timers[i]));
    }

    // Connect the operators to the timer
    mcpwm_oper_handle_t operators[4] = {NULL};
    // Operator must be in the same group as timer
    mcpwm_operator_config_t operator_configs[2] = {{.group_id = 0}, {.group_id = 1}};
    for (uint8_t i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_configs[i % 2], &operators[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], pwm_timers[i % 2]));
    }

    // Create and configure the Comparators
    mcpwm_comparator_config_t cmpr_config = {};
    // Create and configure the Generators
    mcpwm_gen_handle_t generators[4] = {NULL};
    mcpwm_generator_config_t generator_configs[4] = {};
    generator_configs[0].gen_gpio_num = MOTOR_A_CW;
    generator_configs[1].gen_gpio_num = MOTOR_B_CCW;
    generator_configs[2].gen_gpio_num = MOTOR_C_CW;
    generator_configs[3].gen_gpio_num = MOTOR_D_CCW;

    for (uint8_t i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &cmpr_config, &esc_pwm_comparators[i]));
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &generator_configs[i], &generators[i]));
    }

    // On timer period (counter reaches zero): set output high
    for (uint8_t i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generators[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generators[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, esc_pwm_comparators[i], MCPWM_GEN_ACTION_LOW)));
    }

    for (uint8_t i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_timers[i]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwm_timers[i], MCPWM_TIMER_START_NO_STOP));
    }
}

/* esc_pwm_set_duty_cycle()
 * ------------------------
 * Function to control the duty cycle of each of the 4 PWM signal needed
 * for the motor's ESC's.
 *
 * Parameters:
 *    motor - Which of the comparators to adjust
 *    duty_cycle - The value in us after which the PWM signal will go low
 */
void esc_pwm_set_duty_cycle(MotorIndex motor, uint16_t duty_cycle) {

    // Ensure that the motor index given is valid
    if (!(motor < NUMBER_OF_MOTORS)) {
        return;
    }

    // Limit the requested motor speed between the min/max
    duty_cycle = (duty_cycle > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX
                 : duty_cycle < MOTOR_SPEED_MIN ? MOTOR_SPEED_MIN
                                                : duty_cycle;
    switch (motor) {
    case 0:
        duty_cycle -= MOTOR_A_OFFSET;
        break;
    case 1:
        duty_cycle -= MOTOR_B_OFFSET;
        break;
    case 2:
        duty_cycle -= MOTOR_C_OFFSET;
        break;
    case 3:
        duty_cycle -= MOTOR_D_OFFSET;

    default:
        break;
    }

    // Update the comparitor used by the specified motor
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc_pwm_comparators[motor], duty_cycle));
}
