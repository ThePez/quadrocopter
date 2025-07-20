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
#include "bno085_task.h"
#include "common.h"
#include "motors.h"
#include "radio.h"

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t systemTask = NULL;

//////////////////////////////////////////////////////////////////////////////

void radio_task_init(void) {

    SemaphoreHandle_t radioSetupMutex = xSemaphoreCreateMutex();
    RadioParams_t* radioTaskParams = malloc(sizeof(RadioParams_t));
    radioTaskParams->spiHost = HSPI_HOST;
    radioTaskParams->spiMutex = &spiHMutex;
    radioTaskParams->setupMutex = &radioSetupMutex;
    xTaskCreate(&radio_control_task, "RADIO_CON", RADIO_STACK, radioTaskParams, RADIO_PRIO, &radioControlTask);
    xTaskCreate(&radio_receiver_task, "RX_RADIO", RADIO_STACK, radioTaskParams, RADIO_PRIO, &radioReceiverTask);
    xTaskCreate(&radio_transmitter_task, "TX_RADIO", RADIO_STACK, radioTaskParams, RADIO_PRIO, &radioTransmitterTask);
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
    // SPI Bus Setup
    spi_bus_setup(HSPI_HOST);
    // Setup PWM
    esc_pwm_init();

    // Radio tasks Setup
    radio_task_init();

    // Allow the gpio_isr_install to happen from the Radio setup,
    // before the imu task begins

    // IMU task Setup
    bno08x_start_task(); // Start the C++ task

    // Flight Controller
    xTaskCreate((void*) &flight_controller, "FC_Task", SYS_STACK, NULL, SYS_PRIO, &systemTask);
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

    ControlSystem_t controlPIDs = {0};
    // Set default Scalers for the PID structs
    // Angle Outer PID's
    for (uint8_t i = 0; i < 3; i++) {
        controlPIDs.pids[i].kp = 5;
        controlPIDs.pids[i].kd = 0;
        controlPIDs.pids[i].ki = 0;
    }

    // Rate Inner PID's
    for (uint8_t i = 3; i < 6; i++) {
        controlPIDs.pids[i].kp = 5;
        controlPIDs.pids[i].kd = 0;
        controlPIDs.pids[i].ki = 0;
    }

    RemoteSetPoints_t remoteControlInputs = {.throttle = 1300};
    Telemitry_t imuData = {0};
    uint16_t payload[NRF24L01PLUS_TX_PLOAD_WIDTH / 4]; // 8 words

    // Wait until both input queues are created
    while (!radioReceiverQueue || !radioTransmitterQueue || !bno085Queue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Create QueueSet
    uint8_t setLength = RADIO_RECEIVER_QUEUE_LENGTH + BNO085_QUEUE_LENGTH;
    QueueSetHandle_t controlLoopSet = xQueueCreateSet(setLength);
    xQueueAddToSet(radioReceiverQueue, controlLoopSet);
    xQueueAddToSet(bno085Queue, controlLoopSet);
    QueueSetMemberHandle_t xActivatedMember;

    while (1) {

        // This task only delay's based on the QueueSet.
        // Usually the bno085Task will activate it every ~10ms

        if (controlLoopSet) {

            xActivatedMember = xQueueSelectFromSet(controlLoopSet, portMAX_DELAY);

            // Update remote inputs
            if (xActivatedMember == radioReceiverQueue) {
                xQueueReceive(radioReceiverQueue, payload, 0);
                process_remote_data(&remoteControlInputs, &controlPIDs, payload);
            }

            // New position data arrived, run PID loop and update ESC's
            if (xActivatedMember == bno085Queue) {
                xQueueReceive(bno085Queue, &imuData, 0);
                // Run the PID loop
                double errPitch = remoteControlInputs.pitch - imuData.pitchAngle;
                if (fabs(errPitch) < 2.0) {
                    errPitch = 0;
                }

                double errRoll = remoteControlInputs.roll - imuData.rollAngle;
                if (fabs(errRoll) < 2.0) {
                    errRoll = 0;
                }

                double errYaw = remoteControlInputs.yaw - imuData.yawAngle;
                if (fabs(errYaw) < 2.0) {
                    errYaw = 0;
                }

                uint64_t time = esp_timer_get_time();
                double pitchRateGoal = pid_update(&controlPIDs.pids[0], errPitch, time);
                double errPitchRate = pitchRateGoal - imuData.pitchRate;
                double pitchOutput = pid_update(&controlPIDs.pids[1], errPitchRate, time);

                double rollRateGoal = pid_update(&controlPIDs.pids[2], errRoll, time);
                double errRollRate = rollRateGoal - imuData.rollRate;
                double rollOutput = pid_update(&controlPIDs.pids[3], errRollRate, time);

                double yawRateGoal = pid_update(&controlPIDs.pids[4], errYaw, time);
                double errYawRate = yawRateGoal - imuData.yawRate;
                double yawOutput = pid_update(&controlPIDs.pids[5], errYawRate, time);

                // Then update the ESC's
                update_escs(remoteControlInputs.throttle, pitchOutput, rollOutput, yawOutput);
                // uint8_t item[16] = {0x34, 0x56, 0x78, 0x92};
                // xQueueSendToBack(radioTransmitterQueue, item, pdMS_TO_TICKS(5));
            }
        }
    }
}

void process_remote_data(RemoteSetPoints_t* setPoints, ControlSystem_t* system, uint16_t* payload) {
    uint8_t* buffer;
    switch (payload[0]) {
    case PID_MODIFIERS:
        buffer = (uint8_t*) (payload + 1);
        for (uint8_t i = 0; i < 6; i++) {
            system->pids[i].kp = buffer[0];
            system->pids[i].kd = buffer[1];
            system->pids[i].ki = buffer[2];
            buffer += 3; // Move pointer along 3 bytes
        }

        break;

    case SETPOINT_UPDATE:
        setPoints->throttle = (payload[1] / 4.095 + 1000.0);
        // setPoints->pitch = round((payload[2] - 2048.0) / 68.267);
        // setPoints->roll = round((payload[3] - 2048.0) / 68.267);
        // setPoints->yaw = round((payload[4] - 2048.0) / 68.267);
        break;
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
    if (throttle < 1020) {

        for (uint8_t i = 0; i < 4; i++) {
            motorSpeeds[i] = MOTOR_SPEED_MIN;
        }
    } else {

        motorSpeeds[0] = throttle - pitchPID + rollPID - yawPID; // Front left
        motorSpeeds[1] = throttle + pitchPID + rollPID + yawPID; // Rear left
        motorSpeeds[2] = throttle + pitchPID - rollPID - yawPID; // Rear right
        motorSpeeds[3] = throttle - pitchPID - rollPID + yawPID; // Front right
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
double pid_update(PID_t* pid, double error, uint64_t now) {
    if (pid->prevTimeUS == 0) {
        pid->prevTimeUS = now;
        return 0.0;
    }

    long double dt = (now - pid->prevTimeUS) / 1e6;
    pid->prevTimeUS = now;

    double derivative = (error - pid->prevError) / dt;
    pid->prevError = error;
    pid->intergral += error * dt;

    double output = pid->kp * error + pid->ki * pid->intergral + pid->kd * derivative;
    return output;
}
