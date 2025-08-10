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
static PID_t ratePitchPid = {.kp = 1, .ki = 0.00, .kd = 0.00};
static PID_t rateRollPid = {.kp = 0.7, .ki = 0.00, .kd = 0.00};
static PID_t rateYawPid = {.kp = 0.5, .ki = 0.00, .kd = 0.00};
static PID_t anglePitchPid = {.kp = 3, .ki = 0, .kd = 0};
static PID_t angleRollPid = {.kp = 3, .ki = 0, .kd = 0};

// Start in angle mode for leveling
static FlightMode_t current_flight_mode = FLIGHT_MODE_ANGLE;

//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Main entry point of the application.
 *
 * Initializes all subsystems and creates the main FreeRTOS tasks:
 * - Initializes PWM for motor control.
 * - Sets up SPI and radio communication.
 * - Starts the IMU processing task.
 * - Launches the flight controller task.
 *
 * This function is called once at boot by the ESP32 framework.
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
    xTaskCreate(&flight_controller, "FC_Task", SYS_STACK, NULL, SYS_PRIO, &flightController);
}

void set_flight_mode(FlightMode_t mode) {
    if (mode != current_flight_mode) {
        current_flight_mode = mode;

        // Reset PID integrators when switching modes to prevent windup
        pid_reset(&ratePitchPid);
        pid_reset(&rateRollPid);
        pid_reset(&rateYawPid);

        if (mode == FLIGHT_MODE_ANGLE) {
            pid_reset(&anglePitchPid);
            pid_reset(&angleRollPid);
        }

        ESP_LOGI(TAG, "Switched to %s mode", mode == FLIGHT_MODE_ANGLE ? "ANGLE" : "RATE");
    }
}

void flight_controller(void* pvParams) {

    RemoteSetPoints_t remoteControlInputs = {.throttle = 1000};
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

                double pitchRateSetpoint, rollRateSetpoint, yawRateSetpoint;

                if (current_flight_mode == FLIGHT_MODE_ANGLE) {
                    // ANGLE MODE: Cascaded control with outer angle loop

                    // Map remote control inputs to angle setpoints
                    double pitchAngleSetpoint = mapf(remoteControlInputs.pitch, -200.0, 200.0, -MAX_ANGLE, MAX_ANGLE);
                    double rollAngleSetpoint = mapf(remoteControlInputs.roll, -200.0, 200.0, -MAX_ANGLE, MAX_ANGLE);

                    // Outer loop: Angle PID controllers output desired rates
                    double errPitchAngle = pitchAngleSetpoint - imuData.pitchAngle;
                    double errRollAngle = rollAngleSetpoint - imuData.rollAngle;

                    pitchRateSetpoint = pid_update(&anglePitchPid, errPitchAngle, time);
                    rollRateSetpoint = pid_update(&angleRollPid, errRollAngle, time);

                    // Limit the rate setpoints from angle controllers
                    const double MAX_RATE_FROM_ANGLE = 200.0; // Match your remote input range
                    pitchRateSetpoint = constrainf(pitchRateSetpoint, -MAX_RATE_FROM_ANGLE, MAX_RATE_FROM_ANGLE);
                    rollRateSetpoint = constrainf(rollRateSetpoint, -MAX_RATE_FROM_ANGLE, MAX_RATE_FROM_ANGLE);

                    // Yaw is still direct rate control in angle mode
                    yawRateSetpoint = remoteControlInputs.yaw;

                } else {
                    // RATE MODE: Direct rate control (your original setup)
                    pitchRateSetpoint = remoteControlInputs.pitch;
                    rollRateSetpoint = remoteControlInputs.roll;
                    yawRateSetpoint = remoteControlInputs.yaw;
                }

                // Inner loop: Rate PID controllers (always runs)
                double errPitchRate = pitchRateSetpoint - imuData.pitchRate;
                double errRollRate = rollRateSetpoint - imuData.rollRate;
                double errYawRate = yawRateSetpoint - imuData.yawRate;

                double pitchOutput = pid_update(&ratePitchPid, errPitchRate, time);
                double rollOutput = pid_update(&rateRollPid, errRollRate, time);
                double yawOutput = pid_update(&rateYawPid, errYawRate, time);

                // Debug logging
                static uint32_t log_counter = 0;
                if (++log_counter % 10 == 0) { // Log every ~100ms
                    ESP_LOGI(
                        TAG, "Mode: %s, Angles: P=%.02f R=%.02f, Rates: P=%.02f R=%.02f, PID: P=%.02f R=%.02f Y=%.02f",
                        current_flight_mode == FLIGHT_MODE_ANGLE ? "ANGLE" : "RATE", imuData.pitchAngle,
                        imuData.rollAngle, imuData.pitchRate, imuData.rollRate, pitchOutput, rollOutput, yawOutput);
                }

                // Update ESCs
                update_escs(remoteControlInputs.throttle, pitchOutput, rollOutput, yawOutput);
            }
        }
    }
}

void process_remote_data(RemoteSetPoints_t* setPoints, uint16_t* payload) {

    uint8_t i = 0;
    uint8_t* buffer;
    float value;
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
        // ADC values are inverted, all 3 settings are * -1
        // Pitch rate
        value = -mapf(payload[2], ADC_MIN, ADC_MAX, MIN_RATE, MAX_RATE);
        setPoints->pitch = (fabsf(value) < 5) ? 0 : value;
        // Roll rate
        value = -mapf(payload[3], ADC_MIN, ADC_MAX, MIN_RATE, MAX_RATE);
        setPoints->roll = (fabsf(value) < 5) ? 0 : value;
        // Yaw rate
        value = -mapf(payload[4], ADC_MIN, ADC_MAX, MIN_RATE, MAX_RATE);
        setPoints->yaw = (fabsf(value) < 5) ? 0 : value;
        // Flight mode
        set_flight_mode((payload[5] == FLIGHT_MODE_RATE) ? FLIGHT_MODE_RATE : FLIGHT_MODE_ANGLE);
        // Print setpoints
        // printf("Throttle: %f, Pitch: %f, Roll: %f, Yaw: %f\r\n", setPoints->throttle, setPoints->pitch,
        // setPoints->roll,
        //        setPoints->yaw);
        break;
    }
}

void update_escs(uint16_t throttle, double pitchPID, double rollPID, double yawPID) {

    double motorSpeeds[NUMBER_OF_MOTORS];

    // This ensure the drone's motors stay off when the throttle is off
    if (throttle < 1020) {

        for (uint8_t i = 0; i < 4; i++) {
            motorSpeeds[i] = MIN_THROTTLE;
        }
    } else {

        motorSpeeds[0] = constrainf(throttle - pitchPID + rollPID - yawPID, MIN_THROTTLE,
                                    MAX_THROTTLE); // Front left
        motorSpeeds[1] = constrainf(throttle + pitchPID + rollPID + yawPID, MIN_THROTTLE,
                                    MAX_THROTTLE); // Rear left
        motorSpeeds[2] = constrainf(throttle + pitchPID - rollPID - yawPID, MIN_THROTTLE,
                                    MAX_THROTTLE); // Rear right
        motorSpeeds[3] = constrainf(throttle - pitchPID - rollPID + yawPID, MIN_THROTTLE,
                                    MAX_THROTTLE); // Front right
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

void pid_reset(PID_t* pid) {
    if (pid) {
        pid->intergral = 0.0;
        pid->prevError = 0.0;
        pid->prevTimeUS = 0; // Reset timing if your PID structure has this field
    }
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

void remote_data_callback(void* args) {
    xTaskNotifyGive(flightController);
}
