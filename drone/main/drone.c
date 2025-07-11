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
#include "driver/i2c_master.h"
#include "driver/mcpwm_prelude.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Custom Components
#include "hamming.h"
#include "hmc5883l.h"
#include "l3gd20.h"
#include "lis3DH.h"
#include "nrf24l01plus.h"

/////////////////////////////////// Defines //////////////////////////////////

// SPI Pins
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_CLK 14
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_CLK 18

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

#define GYRO_ALPHA 0.40

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
i2c_master_bus_handle_t i2c_bus_setup(void);
void esc_pwm_init(void);
void esc_pwm_set_duty_cycle(MotorIndex motor, uint16_t duty_cycle);
void update_escs(uint16_t throttle, double pitchPID, double rollPID, double yawPID);
double pid_update(PID_t* pid, double error);
void decode_packet(void* input, void* output);
void print_task_stats(void);

void imu_sign_check_task(void);

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t systemTask = NULL;
TaskHandle_t radioTask = NULL;
TaskHandle_t accelTask = NULL;

// Queue's Used by the tasks
QueueHandle_t radioQueue = NULL;
QueueHandle_t imuQueue = NULL;

// Ensure the SPI bus is only setup once
static uint8_t spiVBusInitialised = 0;
static uint8_t spiHBusInitialised = 0;

// Each comparator controls the duty cycle of each PWM signal for the ESC's
mcpwm_cmpr_handle_t esc_pwm_comparators[4] = {NULL};

// PID_t structs for each of the directions
PID_t pitchPID = {.kp = 5, .ki = 0.0, .kd = 0.5};
PID_t rollPID = {.kp = 5, .ki = 0.0, .kd = 0.5};
PID_t yawPID = {.kp = 5, .ki = 0.0, .kd = 0.5};

//////////////////////////////////////////////////////////////////////////////

/* app_main()
 * ----------
 * Main entry point for the application.
 *
 * Creates and launches the three main FreeRTOS tasks:
 *   - flight_controller: Handles the main flight PID loop and motor control.
 *   - radio_task: Handles receiving radio packets from the NRF24L01+.
 *   - imu_task: Reads IMU sensor data (accelerometer + gyro) and fuses it.
 *
 * This function runs once at boot.
 */
void app_main(void) {

    xTaskCreate((void*) &flight_controller, "FC_Task", SYS_STACK, NULL, SYS_PRIO, &systemTask);
    xTaskCreate((void*) &radio_task, "RADIO", RADIO_STACK, NULL, RADIO_PRIO, &radioTask);
    xTaskCreate((void*) &imu_task, "IMU", LIS_STACK, NULL, LIS_PRIO, &accelTask);
    // xTaskCreate((void*) &imu_sign_check_task, "SIGN_CHECK", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
}

/* flight_controller()
 * -------------------
 * The main control loop for the quadcopter.
 *
 * This task waits for new telemetry and radio control data using a QueueSet,
 * then calculates the pitch, roll, and yaw errors, runs the PID control loops,
 * and updates the motor ESC signals accordingly.
 *
 * This ensures the drone maintains its desired attitude and throttle based
 * on remote control inputs and sensor feedback.
 *
 * Dependencies:
 *   - Requires both radioQueue and imuQueue to be created before running.
 *   - Must be run as a FreeRTOS task.
 */
void flight_controller(void) {

    // Varables
    ControlSetPoint_t remoteData = {.throttle = 1000};
    ControlError_t pidError = {0};
    Telemitry_t imuData = {0};

    // Wait until both input queues are created
    while (!radioQueue && !imuQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Setup PWM
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
                // printf("Remote Data Updated\r\n");
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
 * -------------
 * Initializes and runs the NRF24L01+ radio receiver.
 *
 * This task sets up the SPI bus and radio module, then listens for
 * incoming encoded packets from the remote controller.
 * Successfully received packets are decoded and translated into
 * throttle and attitude setpoints, which are sent to the flight controller
 * via a FreeRTOS queue.
 *
 * Runs continuously as a FreeRTOS task.
 *
 * Dependencies:
 *   - Uses spi_bus_setup() and nrf24l01plus_*() driver functions.
 *   - Outputs to radioQueue.
 */
void radio_task(void) {

    // Setup variables
    uint8_t rx_buffer[32];
    uint8_t decodedPacket[16];
    uint16_t inputs[5];
    ControlSetPoint_t remoteInputs = {0};

    while (!radioQueue) {
        // Loop to ensure the radio queue is created
        radioQueue = xQueueCreate(5, sizeof(ControlSetPoint_t));
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Setup hardware
    spi_bus_setup(VSPI_HOST);
    nrf24l01plus_init(VSPI_HOST);
    nrf24l01plus_recieve_mode();

    printf("NRF24L01+ RX listening...\n");

    while (1) {
        if (nrf24l01plus_recieve_packet(rx_buffer)) {

            // Decode packet
            decode_packet((void*) rx_buffer, (void*) decodedPacket);
            memcpy(inputs, decodedPacket, sizeof(inputs));

            // Throttle conversion
            remoteInputs.throttle = (uint16_t) ((float) inputs[0] / 4.096f + 1000.0f);

            // Pitch, Roll & Yaw conversion
            double controlInputs[3];
            for (int i = 1; i < 4; i++) {
                controlInputs[i - 1] = ((float) inputs[i] - 2048.0f) / 68.267f;
            }

            remoteInputs.pitch = controlInputs[0];
            remoteInputs.roll = controlInputs[1];
            remoteInputs.yaw = controlInputs[2];

            // printf("inputs: %d, %f, %f, %f\r\n", remoteInputs.throttle, remoteInputs.pitch, remoteInputs.roll,
            //        remoteInputs.yaw);

            // Send input data to FC
            xQueueSendToFront(radioQueue, &remoteInputs, pdMS_TO_TICKS(5));
            // printf("Data sent over queue\r\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* imu_task()
 * ----------
 * Continuously reads sensor data from the LIS3DH accelerometer
 * and L3GD20 gyroscope.
 *
 * Calculates pitch, roll, and yaw angles using a complementary filter
 * to fuse accelerometer and gyro data. Sends updated telemetry to
 * the flight controller via imuQueue.
 *
 * Runs continuously as a FreeRTOS task.
 *
 * Dependencies:
 *   - Uses lis3dh_init(), gyro_init(), lisReadAxisData(), gyroReadAxisData(),
 *     getPitchAngle(), getRollAngle().
 *   - Outputs to imuQueue.
 */
void imu_task(void) {

    // Setup variables
    Telemitry_t imuData = {0};
    int16_t x, y, z;

    while (!imuQueue) {
        // Loop to ensure the imu queue is created
        imuQueue = xQueueCreate(5, sizeof(imuData));
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    spi_bus_setup(HSPI_HOST);
    lis3dh_init(HSPI_HOST);
    spi_bus_setup(VSPI_HOST);
    gyro_init(VSPI_HOST);
    i2c_master_bus_handle_t bus = i2c_bus_setup();
    magnetometer_init(bus);

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
        double gyroPitch = -1 * y * dt * GYRO_SENSITIVITY;
        double gyroRoll = -1 * x * dt * GYRO_SENSITIVITY;
        double gyroYaw = z * dt * GYRO_SENSITIVITY;

        // Sum up the Yaw angle changes over time, ignoring tiny changes in Z (removes some noise)
        if (z > 50 || z < -50) {
            imuData.yawAngle += gyroYaw;
        }

        // Calculate the Pitch & Roll angles using both sets of data
        imuData.pitchAngle = GYRO_ALPHA * (imuData.pitchAngle + gyroPitch) + (1 - GYRO_ALPHA) * accPitch;
        imuData.rollAngle = GYRO_ALPHA * (imuData.rollAngle + gyroRoll) + (1 - GYRO_ALPHA) * accRoll;

        // Send data to the Flight controller for processing
        if (imuQueue) {
            xQueueSendToFront(imuQueue, &imuData, pdMS_TO_TICKS(1));
        }

        // Repeat this vey quickly
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* spi_bus_setup()
 * ----------------
 * Configures and initializes the specified SPI bus (HSPI or VSPI).
 *
 * Each bus is initialized only once per boot. Pins are configured
 * depending on which host is requested.
 *
 * Parameters:
 *   host - The SPI bus to initialize (HSPI_HOST or VSPI_HOST).
 *
 * Dependencies:
 *   - Uses ESP-IDF SPI Master driver.
 *   - Updates global flags spiHBusInitialised and spiVBusInitialised.
 */
void spi_bus_setup(spi_host_device_t host) {

    if ((host == VSPI_HOST && spiVBusInitialised) || (host == HSPI_HOST && spiHBusInitialised)) {
        return;
    }

    spi_bus_config_t bus_config = {
        .miso_io_num = (host == HSPI_HOST) ? HSPI_MISO : VSPI_MISO,
        .mosi_io_num = (host == HSPI_HOST) ? HSPI_MOSI : VSPI_MOSI,
        .sclk_io_num = (host == HSPI_HOST) ? HSPI_CLK : VSPI_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO));

    if (host == HSPI_HOST) {
        spiHBusInitialised = 1;
    } else if (host == VSPI_HOST) {
        spiVBusInitialised = 1;
    }
}

/* i2c_bus_setup()
 * ---------------
 *
 */
i2c_master_bus_handle_t i2c_bus_setup(void) {

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,    // Default
        .i2c_port = -1,                       // Auto select
        .scl_io_num = GPIO_NUM_22,            // Default
        .sda_io_num = GPIO_NUM_21,            // Default
        .glitch_ignore_cnt = 7,               // Default
        .flags.enable_internal_pullup = true, // Default
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    return bus_handle;
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

    if (motor > 3) {
        return;
    }

    // Limits on the inputs
    duty_cycle = (duty_cycle > 2000) ? 2000 : duty_cycle < 1000 ? 1000 : duty_cycle;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc_pwm_comparators[motor], duty_cycle));
}

/* update_escs()
 * -------------
 * Function to update the PWM duty cycles for all four ESCs based on
 * throttle and the outputs of the pitch, roll, and yaw PID controllers.
 *
 * The mixing formula adjusts each motor’s throttle to achieve the desired
 * pitch, roll, and yaw simultaneously.
 *
 * Parameters:
 *   throttle - Base throttle level (1000–2000 us)
 *   pitchPID - Output of the pitch PID controller
 *   rollPID  - Output of the roll PID controller
 *   yawPID   - Output of the yaw PID controller
 *
 * Globals:
 *   Uses esc_pwm_set_duty_cycle() to update each motor’s PWM.
 */
void update_escs(uint16_t throttle, double pitchPID, double rollPID, double yawPID) {

    double motorA = throttle - pitchPID + rollPID + yawPID; // Front left (CW)
    double motorB = throttle + pitchPID + rollPID - yawPID; // Rear left (CCW)
    double motorC = throttle + pitchPID - rollPID + yawPID; // Rear right (CW)
    double motorD = throttle - pitchPID - rollPID - yawPID; // Front right (CCW)
    uint16_t speeds[4] = {(uint16_t) motorA, (uint16_t) motorB, (uint16_t) motorC, (uint16_t) motorD};

    for (uint8_t i = 0; i < 4; i++) {
        esc_pwm_set_duty_cycle(i, speeds[i]);
        printf("Motor %d: %d | ", i, speeds[i]);
    }
    printf("\r\n");
}

/* pid_update()
 * ------------
 * Runs a single update step of a PID controller.
 *
 * Calculates the proportional, integral, and derivative terms based on the
 * current error and time elapsed since the last update.
 *
 * Parameters:
 *   pid   - Pointer to a PID_t structure with the PID state and constants.
 *   error - Current control error value.
 *
 * Returns:
 *   The calculated control output for the PID loop.
 *
 * Side effects:
 *   Updates the PID's stored previous error, integral sum, and previous timestamp.
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
 * Decodes a received data packet using Hamming decoding.
 *
 * The NRF24L01+ transmits a 32-byte encoded packet (16 words of 16 bits),
 * which this function decodes into the original 16-byte payload.
 *
 * Parameters:
 *   input  - Pointer to the raw received packet buffer.
 *   output - Pointer to the decoded output buffer.
 *
 * Dependencies:
 *   Uses hamming_word_decode() to correct single-bit errors.
 */
void decode_packet(void* input, void* output) {
    uint16_t* in = (uint16_t*) input;
    uint8_t* out = (uint8_t*) output;
    for (uint8_t i = 0; i < NRF24L01PLUS_TX_PLOAD_WIDTH / 2; i++) {
        out[i] = hamming_word_decode(in[i]);
    }
}

/* print_task_stats()
 * ------------------
 * Prints a formatted list of all currently running FreeRTOS tasks,
 * including their state, priority, stack usage, and task ID.
 *
 * Helpful for debugging stack usage and system health.
 *
 * Uses vTaskList() from FreeRTOS and outputs to standard printf().
 */
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

/* IMU sign check test()
 * ---------------------
 * This task reads accelerometer and gyro data, calculates pitch/roll
 * angles from the accelerometer, and prints raw gyro rates.
 * Use this to check if signs and axes match.
 */
void imu_sign_check_task(void) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    spi_bus_setup(HSPI_HOST);
    lis3dh_init(HSPI_HOST);
    spi_bus_setup(VSPI_HOST);
    gyro_init(VSPI_HOST);

    while (1) {
        // Read accelerometer data
        lisReadAxisData(&ax, &ay, &az);
        double accPitch = getPitchAngle(ax, ay, az);
        double accRoll = getRollAngle(ax, ay, az);

        // Read gyro raw rates
        gyroReadAxisData(&gx, &gy, &gz);

        printf("ACC: Pitch = %.2f deg, Roll = %.2f deg | ", accPitch, accRoll);
        printf("GYRO: X = %d, Y = %d, Z = %d (raw dps)\r\n", gx, gy, gz);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
