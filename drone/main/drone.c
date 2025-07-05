/*
 *****************************************************************************
 * File: drone.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

// STD C lib headers
#include <stdio.h>
#include <string.h>

// KConfig header
#include "sdkconfig.h"

// ESP-IDF Prebuilts
#include "driver/mcpwm_prelude.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Custom Components
#include "hamming.h"
#include "l3gd20.h"
#include "lis3DH.h"
#include "nrf24l01plus.h"

/////////////////////////////////// Defines //////////////////////////////////

// SPI Pins
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_CLK 14

// Motor/ESC's GPIO Pins
// Ensure these are sequential as the PWM setup requires this.
// If not possible alter the esc_pwm_init() function.
#define MOTOR_A_CW 32  // Front left
#define MOTOR_B_CCW 33 // Rear left
#define MOTOR_C_CW 34  // Rear right
#define MOTOR_D_CCW 35 // Front right

// Throttle speeds / Duty cycle's in us
#define MOTOR_SPEED_MIN 1000
#define MOTOR_SPEED_MAX 2000

// Stack Sizes
#define SYS_STACK (configMINIMAL_STACK_SIZE * 2)
#define RADIO_STACK (configMINIMAL_STACK_SIZE * 2)
#define LIS_STACK (configMINIMAL_STACK_SIZE * 2)
// Priorities
#define SYS_PRIO (tskIDLE_PRIORITY + 2)
#define RADIO_PRIO (tskIDLE_PRIORITY + 3)
#define LIS_PRIO (tskIDLE_PRIORITY + 4)

///////////////////////////// Structures & Enums /////////////////////////////

typedef enum { MOTOR_A, MOTOR_B, MOROT_C, MOTOR_D } MotorIndex;

///////////////////////////////// Prototypes /////////////////////////////////

// Task Function Prototypes
void system_task(void);
void radio_task(void);
void imu_task(void);

// Setup Function Prototypes
void spi_bus_setup(spi_host_device_t host);
void print_task_stats(void);
void esc_pwm_init(void);
void esc_pwm_set_duty_cycle(uint8_t motor, uint16_t duty_cycle);

////////////////////////////// Global Variables //////////////////////////////

TaskHandle_t systemTask = NULL;
TaskHandle_t radioTask = NULL;
TaskHandle_t accelTask = NULL;
// Queue for remote control inputs
QueueHandle_t radioQueue = NULL;
// Each comparator controls the duty cycle of each PWM signal for the ESC's
mcpwm_cmpr_handle_t esc_pwm_comparators[4] = {NULL};

//////////////////////////////////////////////////////////////////////////////

void app_main(void) {

    xTaskCreate((void*) &system_task, "SYS_CONTROL", SYS_STACK, NULL, SYS_PRIO, &systemTask);
    xTaskCreate((void*) &radio_task, "RADIO", RADIO_STACK, NULL, RADIO_PRIO, &radioTask);
    xTaskCreate((void*) &imu_task, "IMU", LIS_STACK, NULL, LIS_PRIO, &accelTask);

    while (1) {
        // print_task_stats();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void system_task(void) {

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* radio_task()
 * ------------
 * The radio task first initialised the NRF24L01plus module for use as a receiver. After
 * which it then monitors for incomming packets, upon successful complete packet being
 * received it is then added to a queue for processing.
 */
void radio_task(void) {

    // Setup hardware
    spi_bus_setup(HSPI_HOST);
    nrf24l01plus_init(HSPI_HOST);
    nrf24l01plus_recieve_mode();

    // Setup variables
    uint8_t rx_buffer[32];
    radioQueue = xQueueCreate(5, sizeof(rx_buffer));
    printf("NRF24L01+ RX listening...\n");

    while (1) {
        if (nrf24l01plus_recieve_packet(rx_buffer) && radioQueue) {
            xQueueSendToFront(radioQueue, rx_buffer, pdMS_TO_TICKS(5));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Should the loop be broken, cleanup memory.
    vQueueDelete(radioQueue);
    radioTask = NULL;
    radioQueue = NULL;
    vTaskDelete(NULL);
}

void imu_task(void) {

    spi_bus_setup(HSPI_HOST);
    lis3dh_init(HSPI_HOST);
    gyro_init(HSPI_HOST);

    while (1) {

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void spi_bus_setup(spi_host_device_t host) {

    // Ensure the SPI bus is only setup once
    static uint8_t spiBusInitialised = 0;

    if (spiBusInitialised) {
        return;
    }

    spi_bus_config_t bus_config = {
        .miso_io_num = HSPI_MISO,
        .mosi_io_num = HSPI_MOSI,
        .sclk_io_num = HSPI_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO));
    spiBusInitialised = 1;
}

void print_task_stats(void) {

    uint16_t numTasks = uxTaskGetNumberOfTasks();
    uint16_t bufferLength = numTasks * 50;
    char* taskListBuffer = pvPortMalloc(bufferLength * sizeof(char));
    if (taskListBuffer == NULL) {
        return; // Malloc Failed
    }

    vTaskList(taskListBuffer);
    printf("\r\nTask          State  Priority   Stack\tID\r\n");
    printf("=============================================\r\n");
    printf("%s\r\n", taskListBuffer);
    // Free memory
    vPortFree(taskListBuffer);
}

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

    mcpwm_timer_handle_t pwm_timer = NULL;
    mcpwm_timer_config_t pwm_timer_config = {
        .resolution_hz = 1000000,                // 1MHz
        .period_ticks = 20000,                   // 20ms
        .group_id = 0,                           // Group 0
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP, // Count up from 0 then reset to 0
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT   // Default clock source
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&pwm_timer_config, &pwm_timer));

    // Connect the operators to the timer
    mcpwm_oper_handle_t operators[2] = {NULL};
    // Operator must be in the same group as timer
    mcpwm_operator_config_t operator_config = {.group_id = 0};
    for (uint8_t i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], pwm_timer));
    }

    // Create and configure the Comparators
    mcpwm_comparator_config_t cmpr_config = {};
    for (uint8_t i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i % 2], &cmpr_config, &esc_pwm_comparators[i]));
    }

    // Create and configure the Generators
    mcpwm_gen_handle_t generators[4] = {NULL};
    mcpwm_generator_config_t generator_configs[4];
    for (uint8_t i = 0; i < 4; i++) {
        generator_configs[i].gen_gpio_num = i + MOTOR_A_CW;
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[i % 2], &generator_configs[i], &generators[i]));
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

    ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP));
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
void esc_pwm_set_duty_cycle(uint8_t motor, uint16_t duty_cycle) {

    // Limits on the inputs
    if (motor > 3 || duty_cycle > 20000) {
        return;
    }

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc_pwm_comparators[motor], duty_cycle));
}