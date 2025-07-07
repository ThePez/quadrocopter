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
#include <sys/param.h>

// KConfig header
#include "sdkconfig.h"

// ESP-IDF Prebuilts
#include "driver/mcpwm_prelude.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
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
#define MOTOR_A_CW 16  // Front left
#define MOTOR_B_CCW 17 // Rear left
#define MOTOR_C_CW 18  // Rear right
#define MOTOR_D_CCW 19 // Front right

// Throttle speeds / Duty cycle's in us
#define MOTOR_SPEED_MIN 1000
#define MOTOR_SPEED_MAX 2000

#define IMU_ALPHA 0.40

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

typedef struct {
    double kp;           // Proportional scaler
    double ki;           // Intergral scaler
    double kd;           // Derivative scaler
    double intergral;    // running average total of the error
    double prevError;    // previous error
    uint64_t prevTimeUS; // Last update time in us
} PID_t;

typedef struct {
    uint16_t throttle;
    double pitch;
    double roll;
    double yaw;
} ControlSetPoint_t;

typedef struct {
    double errPitch;
    double errRoll;
    double errYaw;
} ControlError_t;

typedef struct {
    double pitchAngle;
    double rollAngle;
    double yawAngle;
    uint64_t prevTime;
} Telemitry_t;

///////////////////////////////// Prototypes /////////////////////////////////

// Task Function Prototypes
void flight_controller(void);
void radio_task(void);
void imu_task(void);

// Setup Function Prototypes
void spi_bus_setup(spi_host_device_t host);
void esc_pwm_init(void);
void esc_pwm_set_duty_cycle(MotorIndex motor, uint16_t duty_cycle);
void update_escs(uint16_t throttle, double pitchPID, double rollPID, double yawPID);
double pid_update(PID_t* pid, double error);
void decode_packet(void* input, void* output);
void print_task_stats(void);

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t systemTask = NULL;
TaskHandle_t radioTask = NULL;
TaskHandle_t accelTask = NULL;

// Queue's Used by the tasks
QueueHandle_t radioQueue = NULL;
QueueHandle_t imuQueue = NULL;

// Each comparator controls the duty cycle of each PWM signal for the ESC's
mcpwm_cmpr_handle_t esc_pwm_comparators[4] = {NULL};

// PID_t structs for each of the directions
PID_t pitchPID = {.kp = 5, .ki = 0.0, .kd = 0.5};
PID_t rollPID = {.kp = 5, .ki = 0.0, .kd = 0.5};
PID_t yawPID = {.kp = 5, .ki = 0.0, .kd = 0.5};

//////////////////////////////////////////////////////////////////////////////

void app_main(void) {

    xTaskCreate((void*) &flight_controller, "FC_Task", SYS_STACK, NULL, SYS_PRIO, &systemTask);
    xTaskCreate((void*) &radio_task, "RADIO", RADIO_STACK, NULL, RADIO_PRIO, &radioTask);
    xTaskCreate((void*) &imu_task, "IMU", LIS_STACK, NULL, LIS_PRIO, &accelTask);

    while (1) {
        // print_task_stats();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* flight_controller()
 * -------------------
 *
 */
void flight_controller(void) {

    // Varables
    ControlSetPoint_t remoteData = {.throttle = 1000};
    ControlError_t pidError = {0};
    Telemitry_t imuData = {0};

    // Wait until both input queues are created
    while (!imuQueue && !radioQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    esc_pwm_init();

    // Create QueueSet
    QueueSetHandle_t controlLoopSet = xQueueCreateSet(10);
    xQueueAddToSet(radioQueue, controlLoopSet);
    xQueueAddToSet(imuQueue, controlLoopSet);
    QueueSetMemberHandle_t xActivatedMember;

    while (1) {

        // This task only delay's based on the QueueSet.
        // Usually the imuTask will activate it every ~5ms

        if (controlLoopSet) {
            xActivatedMember = xQueueSelectFromSet(controlLoopSet, portMAX_DELAY);

            // Update remote inputs
            if (xActivatedMember == radioQueue) {
                xQueueReceive(radioQueue, &remoteData, 0);
                printf("Remote Data Updated\r\n");
            }

            // New position data arrived, run PID loop and update ESC's
            if (xActivatedMember == imuQueue) {
                xQueueReceive(imuQueue, &imuData, 0);
                // Run the PID loop
                pidError.errPitch = remoteData.pitch - imuData.pitchAngle;
                pidError.errRoll = remoteData.roll - imuData.rollAngle;
                pidError.errYaw = remoteData.yaw - imuData.yawAngle;
                double pitchOutput = pid_update(&pitchPID, pidError.errPitch);
                double rollOutput = pid_update(&rollPID, pidError.errRoll);
                double yawOutput = pid_update(&yawPID, pidError.errYaw);
                // printf("PIDs: %f pitch, %f roll, %f yaw\r\n", pitchOutput, rollOutput, yawOutput);
                // Then update the ESC's
                update_escs(remoteData.throttle, pitchOutput, rollOutput, yawOutput);
            }
        }
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
    uint8_t decodedPacket[16];
    float inputs[5];
    ControlSetPoint_t remoteInputs = {0};
    while (!radioQueue) {
        // Loop to ensure the radio queue is created
        radioQueue = xQueueCreate(5, sizeof(remoteInputs));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    printf("NRF24L01+ RX listening...\n");

    while (1) {
        if (nrf24l01plus_recieve_packet(rx_buffer) && radioQueue) {
            // Decode packet
            decode_packet((void*) rx_buffer, (void*) decodedPacket);
            // Move into float space
            memcpy(inputs, decodedPacket, sizeof(decodedPacket));

            remoteInputs.throttle = (uint16_t) inputs[0];
            remoteInputs.pitch = (double) inputs[1];
            remoteInputs.roll = (double) inputs[2];
            remoteInputs.yaw = (double) inputs[3];

            // Send input data to FC
            xQueueSendToFront(radioQueue, &remoteInputs, pdMS_TO_TICKS(5));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* imu_task()
 * ----------
 * Task to read the telemetry data from the LIS3DH and L3GD20 sensors. This data is then
 * sent to the flight controller.
 */
void imu_task(void) {

    // Setup variables
    Telemitry_t imuData = {0};
    int16_t x, y, z;

    while (!imuQueue) {
        // Loop to ensure the imu queue is created
        imuQueue = xQueueCreate(5, sizeof(imuData));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    spi_bus_setup(HSPI_HOST);
    lis3dh_init(HSPI_HOST);
    gyro_init(HSPI_HOST);

    while (1) {

        lisReadAxisData(&x, &y, &z);
        // Get Pitch & Roll angles from accelerometer
        double accPitch = getPitchAngle(x, y, z);
        double accRoll = getRollAngle(x, y, z);

        // Get current time
        uint64_t now = esp_timer_get_time();

        // Get the axis data from the gyroscope
        gyroReadAxisData(&x, &y, &z);
        long double dt = (now - imuData.prevTime) / 1e6;
        imuData.prevTime = now; // Store current time for next run

        // Calculate the changes in angles from the Gyroscope data
        double gyroPitch = x * dt * GYRO_SENSITIVITY;
        double gyroRoll = y * dt * GYRO_SENSITIVITY;
        double gyroYaw = z * dt * GYRO_SENSITIVITY;

        // Sum up the Yaw angle changes over time, ignoring tiny changes in Z (removes some noise)
        if (z > 50 || z < -50) {
            imuData.yawAngle += gyroYaw;
        }

        // Calculate the Pitch & Roll angles using both sets of data
        imuData.pitchAngle = IMU_ALPHA * (imuData.pitchAngle + gyroPitch) + (1 - IMU_ALPHA) * accPitch;
        imuData.rollAngle = IMU_ALPHA * (imuData.rollAngle + gyroRoll) + (1 - IMU_ALPHA) * accRoll;

        // Send data to the Flight controller for processing
        if (imuQueue) {
            xQueueSendToFront(imuQueue, &imuData, pdMS_TO_TICKS(1));
        }

        // Repeat this vey quickly
        vTaskDelay(pdMS_TO_TICKS(5));
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
void esc_pwm_set_duty_cycle(MotorIndex motor, uint16_t duty_cycle) {

    if (motor > 3) {
        return;
    }

    // Limits on the inputs
    duty_cycle = (duty_cycle > 2000) ? 2000 : duty_cycle < 1000 ? 1000 : duty_cycle;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc_pwm_comparators[motor], duty_cycle));
}

/* update_escs()
 * -------------
 *
 */
void update_escs(uint16_t throttle, double pitchPID, double rollPID, double yawPID) {

    double motorA = throttle - pitchPID + rollPID + yawPID; // Front left (CW)
    double motorB = throttle + pitchPID + rollPID - yawPID; // Rear left (CCW)
    double motorC = throttle + pitchPID - rollPID + yawPID; // Rear right (CW)
    double motorD = throttle - pitchPID - rollPID - yawPID; // Front right (CCW)
    uint16_t speeds[4] = {(uint16_t) motorA, (uint16_t) motorB, (uint16_t) motorC, (uint16_t) motorD};

    for (uint8_t i = 0; i < 4; i++) {
        esc_pwm_set_duty_cycle(i, speeds[i]);
        printf("Motor %d, Speed: %d\t", i, speeds[i]);
    }
    printf("\r\n");
}

/* pid_update()
 * ------------
 *
 */
double pid_update(PID_t* pid, double error) {

    uint64_t nowUS = esp_timer_get_time();

    if (pid->prevTimeUS == 0) {
        pid->prevTimeUS = nowUS;
        return 0.0;
    }

    uint64_t dt = nowUS - pid->prevTimeUS;
    pid->prevTimeUS = nowUS;

    double derivative = (error - pid->prevError) / dt;

    pid->prevError = error;
    pid->intergral += error * dt;

    double output = pid->kp * error + pid->ki * pid->intergral + pid->kd * derivative;
    return output;
}

/* decode_packet()
 * ---------------
 *
 */
void decode_packet(void* input, void* output) {
    uint16_t* in = (uint16_t*) input;
    uint8_t* out = (uint8_t*) output;
    for (uint8_t i = 0; i < NRF24L01PLUS_TX_PLOAD_WIDTH / 2; i++) {
        out[i] = hamming_word_decode(in[i]);
    }
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
