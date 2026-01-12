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
#include "common_functions.h"
#include "esp_log.h"

static const char* TAG = "PWM";
static mcpwm_cmpr_handle_t esc_pwm_comparators[4] = {NULL};

#define CHECK_ERR(code, msg)                                                                                           \
    do {                                                                                                               \
        esp_err_t err = (code);                                                                                        \
        if (err != ESP_OK) {                                                                                           \
            ESP_LOGE(TAG, msg);                                                                                        \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

#define CHECK_ERR_NO_LOG(code)                                                                                         \
    do {                                                                                                               \
        esp_err_t err = (code);                                                                                        \
        if (err != ESP_OK) {                                                                                           \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

esp_err_t esc_pwm_init(void) {
    // Create and configure the timers
    mcpwm_timer_handle_t pwm_timers[2] = {NULL};
    for (uint8_t i = 0; i < 2; i++) {
        mcpwm_timer_config_t pwm_timer_config = {
            .resolution_hz = 1000000,                // 1MHz
            .period_ticks = 20000,                   // 20ms
            .group_id = i,                           // Group 0/1
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP, // Count up from 0 then reset to 0
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT   // Default clock source
        };

        CHECK_ERR(mcpwm_new_timer(&pwm_timer_config, &pwm_timers[i]), "Failed to create PWM timer");
    }

    // Connect the operators to the timers
    mcpwm_oper_handle_t operators[4] = {NULL};
    // Operator must be in the same group as timer
    mcpwm_operator_config_t operator_configs[2] = {{.group_id = 0}, {.group_id = 1}};
    for (uint8_t i = 0; i < 4; i++) {
        CHECK_ERR(mcpwm_new_operator(&operator_configs[i % 2], &operators[i]), "Failed to create PWM operator");
        CHECK_ERR(mcpwm_operator_connect_timer(operators[i], pwm_timers[i % 2]), "Failed to connect operator to timer");
    }

    // Create and configure the Comparators and Generators
    mcpwm_comparator_config_t cmpr_config = {0};
    mcpwm_gen_handle_t generators[4] = {NULL};
    mcpwm_generator_config_t generator_configs[4] = {};
    generator_configs[0].gen_gpio_num = MOTOR_A_CW;
    generator_configs[1].gen_gpio_num = MOTOR_B_CCW;
    generator_configs[2].gen_gpio_num = MOTOR_C_CW;
    generator_configs[3].gen_gpio_num = MOTOR_D_CCW;

    for (uint8_t i = 0; i < 4; i++) {
        CHECK_ERR(mcpwm_new_comparator(operators[i], &cmpr_config, &esc_pwm_comparators[i]),
                  "Failed to create PWM comparator");
        CHECK_ERR(mcpwm_new_generator(operators[i], &generator_configs[i], &generators[i]),
                  "Failed to create PWM generator");
    }

    // Configure generator actions: HIGH on timer period, LOW on compare match
    for (uint8_t i = 0; i < 4; i++) {
        CHECK_ERR(mcpwm_generator_set_action_on_timer_event(
                      generators[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                                                  MCPWM_GEN_ACTION_HIGH)),
                  "Failed to set timer action for generator");

        CHECK_ERR(mcpwm_generator_set_action_on_compare_event(
                      generators[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, esc_pwm_comparators[i],
                                                                    MCPWM_GEN_ACTION_LOW)),
                  "Failed to set compare action for generator");

        // Set the initial compare values to MOTOR_SPEED_MIN
        esc_pwm_set_duty_cycle(i, MOTOR_SPEED_MIN);
    }

    // Enable and start the timers
    for (uint8_t i = 0; i < 2; i++) {
        CHECK_ERR(mcpwm_timer_enable(pwm_timers[i]), "Failed to enable PWM timer");
        CHECK_ERR(mcpwm_timer_start_stop(pwm_timers[i], MCPWM_TIMER_START_NO_STOP), "Failed to start PWM timer");
    }

    // Ensure all motors are completely off on startup.
    for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        
        esc_pwm_set_duty_cycle(i, MOTOR_SPEED_MIN);
    }

    ESP_LOGI(TAG, "ESC PWM initialization completed successfully");
    return ESP_OK;
}

esp_err_t esc_pwm_set_duty_cycle(MotorIndex motor, uint16_t duty_cycle) {

    // Ensure that the motor index given is valid
    if (!(motor < NUMBER_OF_MOTORS)) {
        ESP_LOGE(TAG, "Invalid motor index: %d (max: %d)", motor, NUMBER_OF_MOTORS - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // Limit the requested motor speed between the min/max
    duty_cycle = (uint16_t) constrainf(duty_cycle, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

    // Update the comparator used by the specified motor
    CHECK_ERR(mcpwm_comparator_set_compare_value(esc_pwm_comparators[motor], duty_cycle),
              "Failed to set PWM duty cycle for motor");

    return ESP_OK;
}
