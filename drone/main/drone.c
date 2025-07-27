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
#include "common_functions.h"
#include "motors.h"
#include "radio.h"

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t flightController = NULL;

static const char* TAG = "DRONE";

// Set default Scalers for the PID structs
static PID_t ratePitchPid = {.kp = 5, .ki = 0, .kd = 0};
static PID_t rateRollPid = {.kp = 5, .ki = 0, .kd = 0};
static PID_t rateYawPid = {.kp = 5, .ki = 0, .kd = 0};
static PID_t anglePitchPid = {.kp = 5, .ki = 0, .kd = 0};
static PID_t angleRollPid = {.kp = 5, .ki = 0, .kd = 0};

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
    // Setup PWM
    esc_pwm_init();

    // SPI Bus Setup
    spi_bus_setup(HSPI_HOST);
    // Radio tasks Setup
    radio_module_init(&spiHMutex, HSPI_HOST);

    // Allow the gpio_isr_install to happen from the Radio setup,
    // before the imu task begins

    // IMU task Setup
    bno08x_start_task(); // Start the C++ task

    // Flight Controller
    xTaskCreate((void*) &flight_controller, "FC_Task", SYS_STACK, NULL, SYS_PRIO, &flightController);
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

    RemoteSetPoints_t remoteControlInputs = {0};
    Telemitry_t imuData = {0};
    uint16_t payload[RADIO_PAYLOAD_WIDTH / 2]; // 16 words

    // Wait until both input queues are created
    while (!radioReceiverQueue || !radioTransmitterQueue || !bno085Queue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Create QueueSet
    uint8_t setLength = RADIO_QUEUE_LENGTH + BNO085_QUEUE_LENGTH;
    QueueSetHandle_t controlLoopSet = xQueueCreateSet(setLength);
    xQueueAddToSet(radioReceiverQueue, controlLoopSet);
    xQueueAddToSet(bno085Queue, controlLoopSet);
    QueueSetMemberHandle_t xActivatedMember;

    remote_data_return_init(100000);

    while (1) {

        // This task only delay's based on the QueueSet.
        // Usually the bno085Task will activate it every ~10ms

        if (controlLoopSet) {

            xActivatedMember = xQueueSelectFromSet(controlLoopSet, portMAX_DELAY);

            // Update remote inputs
            if (xActivatedMember == radioReceiverQueue) {
                xQueueReceive(radioReceiverQueue, payload, 0);
                process_remote_data(&remoteControlInputs, payload);
            }

            // New position data arrived, run PID loop and update ESC's
            if (xActivatedMember == bno085Queue) {
                xQueueReceive(bno085Queue, &imuData, 0);
                imuData.pitchRate *= RAD_2_DEG;
                imuData.rollRate *= RAD_2_DEG;
                imuData.yawRate *= RAD_2_DEG;
                uint64_t time = esp_timer_get_time();
                
                // Run the PID loop
                double errPitchRate = remoteControlInputs.pitch - imuData.pitchRate;
                double errRollRate = remoteControlInputs.roll - imuData.rollRate;
                double errYawRate = remoteControlInputs.yaw - imuData.yawRate;
                // Rate PID loop
                double pitchOutput = pid_update(&ratePitchPid, errPitchRate, time);
                double rollOutput = pid_update(&rateRollPid, errRollRate, time);
                double yawOutput = pid_update(&rateYawPid, errYawRate, time);

                // Angle PID loop
                // double errPitchAngle = pitchAngleGoal - imuData.pitchAngle;
                // double pitchOutput = pid_update(&anglePitchPid, errPitchAngle, time);
                // double errRollAngle = rollAngleGoal - imuData.rollAngle;
                // double rollOutput = pid_update(&angleRollPid, errRollAngle, time);

                ESP_LOGI(TAG, "Throttle %f PID's: %f, %f, %f", remoteControlInputs.throttle, pitchOutput, rollOutput,
                         yawOutput);

                // Then update the ESC's
                update_escs(remoteControlInputs.throttle, pitchOutput, rollOutput, yawOutput);
            }
        }
    }
}

void process_remote_data(RemoteSetPoints_t* setPoints, uint16_t* payload) {
    uint8_t i = 0;
    uint8_t* buffer;
    switch (payload[0]) {
    case PID_MODIFIERS:
        buffer = (uint8_t*) (payload + 1);
        // Rates
        ratePitchPid.kp = buffer[i++]; // buffer[0]
        ratePitchPid.ki = buffer[i++];
        ratePitchPid.kd = buffer[i++];

        rateRollPid.kp = buffer[i++];
        rateRollPid.ki = buffer[i++];
        rateRollPid.kd = buffer[i++];

        rateYawPid.kp = buffer[i++];
        rateYawPid.ki = buffer[i++];
        rateYawPid.kd = buffer[i++];
        // Angles
        anglePitchPid.kp = buffer[i++];
        anglePitchPid.ki = buffer[i++];
        anglePitchPid.kd = buffer[i++];

        angleRollPid.kp = buffer[i++];
        angleRollPid.ki = buffer[i++];
        angleRollPid.kd = buffer[i++]; // buffer[14]

        break;

    case SETPOINT_UPDATE:
        setPoints->throttle = mapf(payload[1], ADC_MIN, ADC_MAX, MIN_THROTTLE, MAX_THROTTLE);
        setPoints->pitch = mapf(payload[2], ADC_MIN, ADC_MAX, MIN_RATE, MAX_RATE);
        setPoints->roll = mapf(payload[3], ADC_MIN, ADC_MAX, MIN_RATE, MAX_RATE);
        setPoints->yaw = mapf(payload[4], ADC_MIN, ADC_MAX, MIN_RATE, MAX_RATE);
        break;
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
    if (throttle < 1020) {

        for (uint8_t i = 0; i < 4; i++) {
            motorSpeeds[i] = MIN_THROTTLE;
        }
    } else {

        motorSpeeds[0] = constrainf(throttle - pitchPID + rollPID - yawPID, MIN_THROTTLE, MAX_THROTTLE); // - MOTOR_A_OFFSET; // Front left
        motorSpeeds[1] = constrainf(throttle + pitchPID + rollPID + yawPID, MIN_THROTTLE, MAX_THROTTLE); // - MOTOR_B_OFFSET; // Rear left
        motorSpeeds[2] = constrainf(throttle + pitchPID - rollPID - yawPID, MIN_THROTTLE, MAX_THROTTLE); // - MOTOR_C_OFFSET; // Rear right
        motorSpeeds[3] = constrainf(throttle - pitchPID - rollPID + yawPID, MIN_THROTTLE, MAX_THROTTLE); // - MOTOR_D_OFFSET; // Front right
    }

    for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        esc_pwm_set_duty_cycle(i, (uint16_t) motorSpeeds[i]);
    }

    // No waits on these calls. If there hasn't been a signal to send it will be skipped
    if (ulTaskNotifyTake(pdTRUE, 0) == pdTRUE) {
        uint16_t payload[8] = {
            (uint16_t) motorSpeeds[0],
            (uint16_t) motorSpeeds[1],
            (uint16_t) motorSpeeds[2],
            (uint16_t) motorSpeeds[3],
        };

        // No delay, so that if no room the send is skipped
        xQueueSendToBack(radioTransmitterQueue, payload, 0);
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

void remote_data_return_init(int periodUS) {

    esp_timer_create_args_t config = {
        .callback = remote_data_callback,
        .dispatch_method = ESP_TIMER_TASK,
    };
    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&config, &timerHandle);
    esp_timer_start_periodic(timerHandle, periodUS); // 500,000us -> 500ms -> 0.5s
}

/* remote_data_callback()
 * ----------------------
 * Timer callback function to notify the flight controller task that is can send
 * info to the radio transmitter task.
 */
void remote_data_callback(void* args) {
    xTaskNotifyGive(flightController);
}
