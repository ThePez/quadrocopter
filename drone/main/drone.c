/*
 *****************************************************************************
 * File: drone.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "drone.h"
#include "common.h"
#include "imu.h"
#include "motors.h"
#include "radio.h"

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t systemTask = NULL;

// PID_t structs for each of the directions
PID_t pitchPID = {.kp = 40, .ki = 0, .kd = 0.0};
PID_t rollPID = {.kp = 40, .ki = 0, .kd = 0.0};
PID_t yawPID = {.kp = 40, .ki = 0, .kd = 0.0};

//////////////////////////////////////////////////////////////////////////////

void radio_task_init(void) {
    
    SemaphoreHandle_t radioSetupMutex = xSemaphoreCreateMutex();
    RadioParams_t* radioTaskParams = malloc(sizeof(RadioParams_t));
    radioTaskParams->spiHost = VSPI_HOST;
    radioTaskParams->spiMutex = &spiVMutex;
    radioTaskParams->setupMutex = &radioSetupMutex;
    xTaskCreate((void*) &radio_control_task, "RADIO_CON", RADIO_STACK, radioTaskParams, RADIO_PRIO, &radioControlTask);
    xTaskCreate((void*) &radio_receiver_task, "RX_RADIO", RADIO_STACK, radioTaskParams, RADIO_PRIO, &radioReceiverTask);
    xTaskCreate((void*) &radio_transmitter_task, "TX_RADIO", RADIO_STACK, radioTaskParams, RADIO_PRIO,
                &radioTransmitterTask);
}

void imu_task_init(i2c_master_bus_handle_t* i2cBus) {

    ImuParams_t* imuTaskParams = malloc(sizeof(ImuParams_t));
    imuTaskParams->i2cHost = i2cBus;
    imuTaskParams->i2cMutex = &i2cMutex;
    imuTaskParams->spiHMutex = &spiHMutex;
    imuTaskParams->spiVMutex = &spiVMutex;
    xTaskCreate((void*) &imu_task, "IMU", IMU_STACK, imuTaskParams, IMU_PRIO, &imuTask);
}

/* app_main()
 * ----------
 * Main entry point for the application.
 *
 * Creates and launches the three main FreeRTOS tasks:
 *   - flight_controller: Handles the main flight PID loop and motor control.
 *   - radio_receiver_task: Handles receiving radio packets from the NRF24L01+.
 *   - imu_task: Reads IMU sensor data (accelerometer + gyro) and fuses it.
 *
 * This function runs once at boot.
 */
void app_main(void) {
    // SPI/I2C Bus Setup
    spi_bus_setup(VSPI_HOST);
    spi_bus_setup(HSPI_HOST);
    i2c_master_bus_handle_t* i2cBus = i2c_bus_setup();

    // Setup PWM
    esc_pwm_init();
    // Radio Setup
    radio_task_init();
    // IMU Setup
    imu_task_init(i2cBus);

    // Flight Controller
    xTaskCreate((void*) &flight_controller, "FC_Task", SYS_STACK, NULL, SYS_PRIO, &systemTask);

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
 *   - Requires both radioReceiverQueue and imuQueue to be created before running.
 *   - Must be run as a FreeRTOS task.
 */
void flight_controller(void) {

    Telemitry_t imuData = {0};
    RemoteSetPoints_t remoteData = {.throttle = 1000};
    uint16_t payload[NRF24L01PLUS_TX_PLOAD_WIDTH / 4]; // 8 words

    // Wait until both input queues are created
    while (!radioReceiverQueue || !radioTransmitterQueue || !imuQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Create QueueSet
    uint8_t setLength = RADIO_RECEIVER_QUEUE_LENGTH + IMU_QUEUE_LENGTH;
    QueueSetHandle_t controlLoopSet = xQueueCreateSet(setLength);
    xQueueAddToSet(radioReceiverQueue, controlLoopSet);
    xQueueAddToSet(imuQueue, controlLoopSet);
    QueueSetMemberHandle_t xActivatedMember;

    while (1) {

        // This task only delay's based on the QueueSet.
        // Usually the imuTask will activate it every ~5ms

        if (controlLoopSet) {

            xActivatedMember = xQueueSelectFromSet(controlLoopSet, portMAX_DELAY);

            // Update remote inputs
            if (xActivatedMember == radioReceiverQueue) {
                xQueueReceive(radioReceiverQueue, payload, 0);
                // Convert remote ADC for throttle
                remoteData.throttle = throttle_adc_convert(payload[0]);
                remoteData.pitch = angle_adc_convert(payload[1]);
                remoteData.roll = angle_adc_convert(payload[2]);
                remoteData.yaw = angle_adc_convert(payload[3]);
            }

            // New position data arrived, run PID loop and update ESC's
            if (xActivatedMember == imuQueue) {
                xQueueReceive(imuQueue, &imuData, 0);
                // Run the PID loop
                double errPitch = remoteData.pitch - imuData.pitchAngle;
                if (fabs(errPitch) < 2.0) {
                    errPitch = 0;
                }

                double errRoll = remoteData.roll - imuData.rollAngle;
                if (fabs(errRoll) < 2.0) {
                    errRoll = 0;
                }

                double errYaw = remoteData.yaw - imuData.yawAngle;
                if (fabs(errYaw) < 2.0) {
                    errYaw = 0;
                }
                // printf("Perr: %f, Rerr: %f, Yerr: %f\r\n", pidError.errPitch, pidError.errRoll, pidError.errYaw);
                double pitchOutput = pid_update(&pitchPID, errPitch);
                double rollOutput = pid_update(&rollPID, errRoll);
                double yawOutput = pid_update(&yawPID, errYaw);
                // Then update the ESC's
                update_escs(remoteData.throttle, pitchOutput, rollOutput, yawOutput);
                // uint8_t item[16] = {0x34, 0x56, 0x78, 0x92};
                // xQueueSendToBack(radioTransmitterQueue, item, pdMS_TO_TICKS(5));
            }
        }
    }
}

/* throttle_adc_convert()
 * ----------------------
 * Converts a raw 12-bit ADC value (0 to 4095) into a throttle signal range
 * from 1000 to 2000.
 *
 * This maps the ADC input proportionally to the expected PWM microsecond
 * range for standard ESCs or servos.
 *
 * Dependencies:
 *   - Assumes the ADC provides an unsigned 12-bit value.
 *   - No external dependencies.
 */
double throttle_adc_convert(uint16_t value) {
    return ((double) value) / 4.095 + 1000;
}

/* angle_adc_convert()
 * -------------------
 * Converts a raw 12-bit ADC value (0 to 4095) into an angle measurement
 * ranging approximately from -30 to +30 degrees.
 *
 * The conversion recenters the input to a signed range (-2048 to 2047)
 * and scales it by a fixed factor to produce a meaningful angle.
 * A dead zone is applied: any value within ±2 degrees is set to 0
 * to filter out small noise near the neutral point.
 *
 * Dependencies:
 *   - Assumes the ADC provides an unsigned 12-bit value centered at 2048.
 *   - Uses fabs() from math.h.
 */
double angle_adc_convert(uint16_t value) {
    double angle = ((double) value - 2048.0) / 68.267;
    if (fabs(angle) < 2) {
        angle = 0.0;
    }

    return angle;
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

    double motorSpeeds[NUMBER_OF_MOTORS];

    // This ensure the drone's motors stay off when the throttle is off
    if (throttle < 1040) {

        for (uint8_t i = 0; i < 4; i++) {
            motorSpeeds[i] = MOTOR_SPEED_MIN;
        }

    } else {

        // for (uint8_t i = 0; i < 4; i++) {
        //     motorSpeeds[i] = throttle;
        // }

        // Sign matrix: rows = motors, columns = [pitch, roll, yaw]
        // Front Left  => [-1,  1,  1]
        // Rear Left   => [ 1,  1, -1]
        // Rear Right  => [ 1, -1,  1]
        // Front Right => [-1, -1, -1]

        double changes[NUMBER_OF_MOTORS];
        const int sign_matrix[NUMBER_OF_MOTORS][3] = {{-1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {-1, -1, -1}};
        for (int i = 0; i < NUMBER_OF_MOTORS; i++) {

            // Calcuate the alteration of te base throttle speed from the PID inputs
            changes[i] = sign_matrix[i][0] * pitchPID + sign_matrix[i][1] * rollPID + sign_matrix[i][2] * yawPID;

            // Cap the adjustment to no more then MAX_MOTOR_ADJUSTMENT
            changes[i] = (changes[i] > MAX_MOTOR_ADJUSTMENT)    ? MAX_MOTOR_ADJUSTMENT
                         : (changes[i] < -MAX_MOTOR_ADJUSTMENT) ? -MAX_MOTOR_ADJUSTMENT
                                                                : changes[i];

            // Calculate the new motor speed
            motorSpeeds[i] = throttle + changes[i];
        }
    }

    for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        esc_pwm_set_duty_cycle(i, (uint16_t) motorSpeeds[i]);
        if (i == 3) {
            // printf("Motor %d: %d\r\n", i + 1, (uint16_t) motorSpeeds[i]);
        } else {
            // printf("Motor %d: %d\t", i + 1, (uint16_t) motorSpeeds[i]);
        }
    }
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

    long double dt = (nowUS - pid->prevTimeUS) / 1e6;
    pid->prevTimeUS = nowUS;

    double derivative = (error - pid->prevError) / dt;

    pid->prevError = error;
    pid->intergral += error * dt;

    double output = pid->kp * error + pid->ki * pid->intergral + pid->kd * derivative;
    return output;
}
