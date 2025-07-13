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

    xTaskCreate((void*) &flight_controller, "FC_Task", SYS_STACK, NULL, SYS_PRIO, &systemTask);
    xTaskCreate((void*) &radio_receiver_task, "RX_RADIO", RADIO_STACK, NULL, RADIO_PRIO + 1, &radioReceiverTask);
    xTaskCreate((void*) &radio_transmitter_task, "TX_RADIO", RADIO_STACK, NULL, RADIO_PRIO, &radioTransmitterTask);
    xTaskCreate((void*) &imu_task, "IMU", IMU_STACK, NULL, IMU_PRIO, &imuTask);
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

    // Varables
    ControlSetPoint_t remoteData = {.throttle = 1000};
    ControlError_t pidError = {0};
    Telemitry_t imuData = {0};

    // Wait until both input queues are created
    while (!radioReceiverQueue || !imuQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Setup PWM
    esc_pwm_init();

    // Create QueueSet
    QueueSetHandle_t controlLoopSet = xQueueCreateSet(10);
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
                xQueueReceive(radioReceiverQueue, &remoteData, 0);
            }

            // New position data arrived, run PID loop and update ESC's
            if (xActivatedMember == imuQueue) {
                xQueueReceive(imuQueue, &imuData, 0);
                // Run the PID loop
                pidError.errPitch = remoteData.pitch - imuData.pitchAngle;
                if (fabs(pidError.errPitch) < 2.0) {
                    pidError.errPitch = 0;
                }

                pidError.errRoll = remoteData.roll - imuData.rollAngle;
                if (fabs(pidError.errRoll) < 2.0) {
                    pidError.errRoll = 0;
                }

                pidError.errYaw = remoteData.yaw - imuData.yawAngle;
                if (fabs(pidError.errYaw) < 2.0) {
                    pidError.errYaw = 0;
                }
                // printf("Perr: %f, Rerr: %f, Yerr: %f\r\n", pidError.errPitch, pidError.errRoll, pidError.errYaw);
                double pitchOutput = pid_update(&pitchPID, pidError.errPitch);
                double rollOutput = pid_update(&rollPID, pidError.errRoll);
                double yawOutput = pid_update(&yawPID, pidError.errYaw);
                // Then update the ESC's
                update_escs(remoteData.throttle, pitchOutput, rollOutput, yawOutput);
                uint8_t item[16] = {0x34, 0x56, 0x78, 0x92};
                xQueueSendToBack(radioTransmitterQueue, item, pdMS_TO_TICKS(5));
            }
        }
    }
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
            printf("Motor %d: %d\r\n", i + 1, (uint16_t) motorSpeeds[i]);
        } else {
            printf("Motor %d: %d\t", i + 1, (uint16_t) motorSpeeds[i]);
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
