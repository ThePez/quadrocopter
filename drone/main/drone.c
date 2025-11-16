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

#include "common_functions.h"
#include "motors.h"
#include "radio.h"

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t flightController = NULL;

static const char* TAG = "DRONE";

// Set default Scalers for the PID structs
static PID_t ratePitchPid = {.kp = 0.8, .ki = 0.00, .kd = 0.001};
static PID_t rateRollPid = {.kp = 0.6, .ki = 0.00, .kd = 0.001};
static PID_t rateYawPid = {.kp = 0.1, .ki = 0.00, .kd = 0.00};
static PID_t anglePitchPid = {.kp = 5, .ki = 0, .kd = 0};
static PID_t angleRollPid = {.kp = 5, .ki = 0, .kd = 0};

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
    imu_init(); // Start the C++ task

    // Flight Controller
    xTaskCreate(&flight_controller, "FC_Task", SYS_STACK, NULL, SYS_PRIO, &flightController);
}

void set_flight_mode(FlightMode_t mode, BlackBox_t* box) {
    if (mode != box->mode) {
        box->mode = mode;

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

BlackBox_t* black_box_init(void) {

    BlackBox_t* box = pvPortMalloc(sizeof(BlackBox_t));
    if (!box) {
        ESP_LOGE(TAG, "Black Box init failed");
        return NULL;
    }

    box->imuData = pvPortMalloc(sizeof(Telemitry_t));
    if (box->imuData) {
        memset(box->imuData, 0, sizeof(Telemitry_t));
    } else {
        ESP_LOGE(TAG, "IMU Data init failed");
        goto free_box;
    }

    box->setPoints = pvPortMalloc(sizeof(RemoteSetPoints_t));
    if (box->setPoints) {
        memset(box->setPoints, 0, sizeof(RemoteSetPoints_t));
        box->setPoints->throttle = MIN_THROTTLE;
    } else {
        ESP_LOGE(TAG, "Remote Setpoint data failed");
        goto free_imu;
    }

    box->motorInputs = pvPortMalloc(sizeof(PIDOutputs_t));
    if (box->motorInputs) {
        memset(box->motorInputs, 0, sizeof(PIDOutputs_t));
    } else {
        ESP_LOGE(TAG, "Motor input data failed");
        goto free_setpoint;
    }

    box->motorOutputs = pvPortMalloc(sizeof(MotorPeriods_t));
    if (box->motorOutputs) {
        memset(box->motorOutputs, 0, sizeof(MotorPeriods_t));
    } else {
        ESP_LOGE(TAG, "Motor Period data failed");
        goto free_motor;
    }

    box->mode = FLIGHT_MODE_ANGLE;
    return box;

    /* Clean up paths */
free_motor:
    vPortFree(box->motorInputs);
free_setpoint:
    vPortFree(box->setPoints);
free_imu:
    vPortFree(box->imuData);
free_box:
    vPortFree(box);
    return NULL;
}

void flight_controller(void* pvParams) {

    BlackBox_t* box = black_box_init();
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

    remote_data_return_init(100000, box);

    while (1) {

        // This task only delay's based on the QueueSet.
        // Usually the bno085Task will activate it every ~10ms

        if (controlLoopSet) {

            xActivatedMember = xQueueSelectFromSet(controlLoopSet, portMAX_DELAY);

            // Update remote inputs
            if (xActivatedMember == radioReceiverQueue) {
                xQueueReceive(radioReceiverQueue, payload, 0);
                process_remote_data(box, payload);
            }

            // New position data arrived, run PID loop and update ESC's
            if (xActivatedMember == bno085Queue) {
                xQueueReceive(bno085Queue, box->imuData, 0);
                box->imuData->pitchRate *= RAD_2_DEG;
                box->imuData->rollRate *= RAD_2_DEG;
                box->imuData->yawRate *= RAD_2_DEG;
                uint64_t time = esp_timer_get_time();

                double pitchRateSetpoint, rollRateSetpoint, yawRateSetpoint;

                if (box->mode == FLIGHT_MODE_ANGLE) {
                    // ANGLE MODE: Cascaded control with outer angle loop

                    // Map remote control inputs to angle setpoints
                    double pitchAngleSetpoint = mapf(box->setPoints->pitch, -MAX_RATE, MAX_RATE, -MAX_ANGLE, MAX_ANGLE);
                    double rollAngleSetpoint = mapf(box->setPoints->roll, -MAX_RATE, MAX_RATE, -MAX_ANGLE, MAX_ANGLE);

                    // Outer loop: Angle PID controllers output desired rates
                    double errPitchAngle = pitchAngleSetpoint - box->imuData->pitchAngle;
                    double errRollAngle = rollAngleSetpoint - box->imuData->rollAngle;

                    pitchRateSetpoint = pid_update(&anglePitchPid, errPitchAngle, time);
                    rollRateSetpoint = pid_update(&angleRollPid, errRollAngle, time);

                    // Limit the rate setpoints from angle controllers
                    pitchRateSetpoint = constrainf(pitchRateSetpoint, -MAX_RATE, MAX_RATE);
                    rollRateSetpoint = constrainf(rollRateSetpoint, -MAX_RATE, MAX_RATE);

                    // Yaw is still direct rate control in angle mode
                    yawRateSetpoint = box->setPoints->yaw;

                } else {
                    // RATE MODE: Direct rate control
                    pitchRateSetpoint = box->setPoints->pitch;
                    rollRateSetpoint = box->setPoints->roll;
                    yawRateSetpoint = box->setPoints->yaw;
                }

                // Inner loop: Rate PID controllers (always runs)
                double errPitchRate = pitchRateSetpoint - box->imuData->pitchRate;
                double errRollRate = rollRateSetpoint - box->imuData->rollRate;
                double errYawRate = yawRateSetpoint - box->imuData->yawRate;

                box->motorInputs->pitchPID = pid_update(&ratePitchPid, errPitchRate, time);
                box->motorInputs->rollPID = pid_update(&rateRollPid, errRollRate, time);
                box->motorInputs->yawPID = pid_update(&rateYawPid, errYawRate, time);

                // Debug logging
                static uint32_t log_counter = 0;
                if (++log_counter % 10 == 0) { // Log every ~100ms
                    ESP_LOGI(TAG, "Mode: %s, PID: P=%.02f R=%.02f Y=%.02f", box->mode ? "ANGLE" : "RATE",
                             box->motorInputs->pitchPID, box->motorInputs->rollPID, box->motorInputs->yawPID);
                }

                // Update ESCs
                update_escs(box);
            }
        }
    }
}

void process_remote_data(BlackBox_t* box, uint16_t* payload) {

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
        box->setPoints->throttle = mapf(payload[1], ADC_MIN, ADC_MAX, MIN_THROTTLE, MAX_THROTTLE);
        // Pitch rate
        value = -mapf(payload[2], ADC_MIN, ADC_MAX, MIN_RATE, MAX_RATE);
        box->setPoints->pitch = (fabsf(value) < 5) ? 0 : value;
        // Roll rate
        value = -mapf(payload[3], ADC_MIN, ADC_MAX, MIN_RATE, MAX_RATE);
        box->setPoints->roll = (fabsf(value) < 5) ? 0 : value;
        // Yaw rate
        value = mapf(payload[4], ADC_MIN, ADC_MAX, MIN_RATE, MAX_RATE);
        box->setPoints->yaw = (fabsf(value) < 5) ? 0 : value;
        // Flight mode
        set_flight_mode((payload[5] == FLIGHT_MODE_RATE) ? FLIGHT_MODE_RATE : FLIGHT_MODE_ANGLE, box);
        break;
    }
}

void update_escs(BlackBox_t* box) {

    double throttle = box->setPoints->throttle;
    double pitchPID = box->motorInputs->pitchPID;
    double rollPID = box->motorInputs->rollPID;
    double yawPID = box->motorInputs->yawPID;
    double speed = MIN_THROTTLE;

    // Ensure the motors are off on low throttle inputs
    if (throttle < 1020) {

        box->motorOutputs->motorA = speed;
        box->motorOutputs->motorB = speed;
        box->motorOutputs->motorC = speed;
        box->motorOutputs->motorD = speed;
    } else {

        // Front left
        speed = throttle - pitchPID + rollPID - yawPID;
        box->motorOutputs->motorA = constrainf(speed, MIN_THROTTLE, MAX_THROTTLE);
        // Rear left
        speed = throttle + pitchPID + rollPID + yawPID;
        box->motorOutputs->motorB = constrainf(speed, MIN_THROTTLE, MAX_THROTTLE);
        // Rear right
        speed = throttle + pitchPID - rollPID - yawPID;
        box->motorOutputs->motorC = constrainf(speed, MIN_THROTTLE, MAX_THROTTLE);
        // Front right
        speed = throttle - pitchPID - rollPID + yawPID;
        box->motorOutputs->motorD = constrainf(speed, MIN_THROTTLE, MAX_THROTTLE);
    }

    esc_pwm_set_duty_cycle(MOTOR_A, (uint16_t) box->motorOutputs->motorA);
    esc_pwm_set_duty_cycle(MOTOR_B, (uint16_t) box->motorOutputs->motorB);
    esc_pwm_set_duty_cycle(MOTOR_C, (uint16_t) box->motorOutputs->motorC);
    esc_pwm_set_duty_cycle(MOTOR_D, (uint16_t) box->motorOutputs->motorD);
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

void remote_data_return_init(int periodUS, BlackBox_t* box) {

    esp_timer_create_args_t config = {
        .callback = remote_data_callback,  // Function to execute
        .dispatch_method = ESP_TIMER_TASK, // Where the function is called from
        .arg = box                         // Input argument
    };
    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&config, &timerHandle);
    esp_timer_start_periodic(timerHandle, periodUS); // 500,000us -> 500ms -> 0.5s
}

void remote_data_callback(void* args) {
    BlackBox_t* box = (BlackBox_t*) args;

    int16_t package[RADIO_PAYLOAD_WIDTH / 2]; // 16 words

    // Angles: pitch, roll, yaw     = 3
    // Rates: pitch, roll, yaw      = 3
    // Flight mode                  = 1
    // PID outputs pitch, roll, yaw = 3
    // Motor periods A, B, C, D     = 4
    // Total                       = 14

    // Angles
    package[0] = (int16_t) box->imuData->pitchAngle;
    package[1] = (int16_t) box->imuData->rollAngle;
    package[2] = (int16_t) box->imuData->yawAngle;
    // Rate
    package[3] = (int16_t) box->imuData->pitchRate;
    package[4] = (int16_t) box->imuData->rollRate;
    package[5] = (int16_t) box->imuData->yawRate;
    // Flight mode
    package[6] = (int16_t) box->mode;
    // PID outputs
    package[7] = (int16_t) box->motorInputs->pitchPID;
    package[8] = (int16_t) box->motorInputs->rollPID;
    package[9] = (int16_t) box->motorInputs->yawPID;
    // Motor periods
    package[10] = box->motorOutputs->motorA;
    package[11] = box->motorOutputs->motorB;
    package[12] = box->motorOutputs->motorC;
    package[13] = box->motorOutputs->motorD;

    xQueueSendToBack(radioTransmitterQueue, package, 0);
}
