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
#include "nrf24l01plus.h"
#include "mcp3208.h"
#include "motors.h"

#define TAG "DRONE"
#define FAILSAFE_TIMEOUT_US 1000000 // 1 second

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t flightController = NULL;

// Set default Scalers for the PID structs
static PID_t ratePitchPid =     {.kp = 0.3, .ki = 0.000, .kd = 0.00, .intLimit = 5.0};
static PID_t rateRollPid =      {.kp = 0.3, .ki = 0.000, .kd = 0.00, .intLimit = 5.0};
static PID_t rateYawPid =       {.kp = 0.0, .ki = 0.000, .kd = 0.00, .intLimit = 1.5};

static PID_t anglePitchPid =    {.kp = 1.5, .ki = 0.000, .kd = 0.00, .intLimit = 5.0};
static PID_t angleRollPid =     {.kp = 1.5, .ki = 0.000, .kd = 0.00, .intLimit = 5.0};

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
    /* Physical Hardware setup */
    // Setup PWM
    esc_pwm_init();
    // SPI Bus Setup for radio and mcp3208
    spi_bus_setup(HSPI_HOST);

    /* Helper tasks */
    radio_module_init(&spiHMutex, HSPI_HOST);
    mcpx_task_init(&spiHMutex, 0x02, HSPI_HOST, 33); // DRONE MCP3208 CS pin is 33

    // the gpio_isr_install happens in radio setup -> it has been disabled in the BNO085x driver init
    imu_init(); // Start the C++ task

    /* Main control Task */
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
        goto free_pid;
    }

    box->mode = FLIGHT_MODE_RATE;
    box->connectedToRemote = 0;
    box->failsafeActive = 1;
    return box;

    /* Clean up paths */
free_pid:
    vPortFree(box->motorInputs);
free_setpoint:
    vPortFree(box->setPoints);
free_imu:
    vPortFree(box->imuData);
free_box:
    vPortFree(box);
    return NULL;
}

void black_box_destroy(BlackBox_t* box) {
    // Free the motor speed container
    vPortFree(box->motorOutputs);
    // Free the PID inputs
    vPortFree(box->motorInputs);
    // Free the remote setpoints
    vPortFree(box->setPoints);
    // Free the imu container
    vPortFree(box->imuData);
    // Free the container
    vPortFree(box);
}

void flight_controller(void* pvParams) {

    // Setup the container
    BlackBox_t* box = black_box_init();
    if (box == NULL) {
        vTaskDelete(NULL);
    }

    // Radio payload container
    uint16_t* payload = pvPortMalloc(sizeof(uint16_t) * (RADIO_PAYLOAD_WIDTH / 2));
    if (payload == NULL) {
        black_box_destroy(box);
        vTaskDelete(NULL);
    }

    // Wait until both input queues are created
    while (!radioReceiverQueue || !radioTransmitterQueue || !imuQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Failsafe tracking - last time we got a SETPOINT_UPDATE
    uint64_t lastRemoteUpdateTime = esp_timer_get_time();
    
    // Create QueueSet
    uint8_t setLength = RADIO_QUEUE_LENGTH + BNO085_QUEUE_LENGTH;
    QueueSetHandle_t controlLoopSet = xQueueCreateSet(setLength);
    xQueueAddToSet(radioReceiverQueue, controlLoopSet);
    xQueueAddToSet(imuQueue, controlLoopSet);
    QueueSetMemberHandle_t xActivatedMember;

    // Initialise a callback task for returning info back to the remote
    timer_task_callback_init(100000, box, remote_data_callback); // Interval of 100ms
    timer_task_callback_init(50000, box, battery_callback); // Interval of 50ms

    while (1) {

        // This task only delay's based on the QueueSet.
        // Usually the imu will activate it every ~10ms

        if (controlLoopSet) {

            xActivatedMember = xQueueSelectFromSet(controlLoopSet, portMAX_DELAY);
            uint64_t now = esp_timer_get_time();

            // Update remote inputs
            if (xActivatedMember == radioReceiverQueue) {
                // Get remote set point data
                xQueueReceive(radioReceiverQueue, payload, 0);
                // Process it
                process_remote_data(box, payload);
                // Update flags
                lastRemoteUpdateTime = now;
                box->connectedToRemote = 1;
                box->failsafeActive = 0;
            }

            // New position data arrived, run PID loop and update ESC's
            if (xActivatedMember == imuQueue) {
                // Get telemitry data from IMU
                xQueueReceive(imuQueue, box->imuData, 0);

                // Is the remote armed / connected?
                if(now - lastRemoteUpdateTime > FAILSAFE_TIMEOUT_US) {
                    failsafe(box);
                    continue;
                }
                
                // Normal PID update
                process_positional_data(box);
            }
        }
    }
}

void failsafe(BlackBox_t* box) {
    if (box->failsafeActive) {
        return;
    }

    ESP_LOGE(TAG, "FAILSAFE TRIGGERED: No radio link for at least 2s");
    box->failsafeActive = 1;
    box->connectedToRemote = 0;

    // Force motors to MIN_THROTTLE and skip PID update
    box->motorOutputs->motorA = MIN_THROTTLE;
    box->motorOutputs->motorB = MIN_THROTTLE;
    box->motorOutputs->motorC = MIN_THROTTLE;
    box->motorOutputs->motorD = MIN_THROTTLE;

    esc_pwm_set_duty_cycle(MOTOR_A, (uint16_t) box->motorOutputs->motorA);
    esc_pwm_set_duty_cycle(MOTOR_B, (uint16_t) box->motorOutputs->motorB);
    esc_pwm_set_duty_cycle(MOTOR_C, (uint16_t) box->motorOutputs->motorC);
    esc_pwm_set_duty_cycle(MOTOR_D, (uint16_t) box->motorOutputs->motorD);
}

void process_positional_data(BlackBox_t* box) {

    uint64_t time = box->imuData->prevTime;
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
    // static uint32_t log_counter = 0;
    // if (++log_counter % 10 == 0) { // Log every ~100ms
    //     ESP_LOGI(TAG, "Mode: %s, PID: P=%.02f R=%.02f Y=%.02f", box->mode ? "ANGLE" : "RATE",
    //              box->motorInputs->pitchPID, box->motorInputs->rollPID, box->motorInputs->yawPID);
    // }

    // Update ESCs
    update_escs(box);
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
        box->setPoints->throttle = mapf(payload[1], 0, ADC_MAX, MIN_THROTTLE, MAX_THROTTLE);
        // Pitch rate
        value = -mapf(payload[2], 0, ADC_MAX, -MAX_RATE, MAX_RATE);
        box->setPoints->pitch = (fabsf(value) < 5) ? 0 : value;
        // Roll rate
        value = -mapf(payload[3], 0, ADC_MAX, -MAX_RATE, MAX_RATE);
        box->setPoints->roll = (fabsf(value) < 5) ? 0 : value;
        // Yaw rate
        value = mapf(payload[4], 0, ADC_MAX, -MAX_RATE, MAX_RATE);
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
        pid->prevError = error;
        return 0.0;
    }

    long double dt = (now - pid->prevTimeUS) / 1e6;
    pid->prevTimeUS = now;

    // Proportional term
    double pTerm = pid->kp * error;

    // Integral term with anti-windup clamping
    pid->intergral += error * dt;

    // Clamp the integral accumulator
    if (pid->intergral > pid->intLimit) {
        pid->intergral = pid->intLimit;
    } else if (pid->intergral < -pid->intLimit) {
        pid->intergral = -pid->intLimit;
    }

    double iTerm = pid->ki * pid->intergral;

    // Derivative term (basic - could add filtering later)
    double derivative = (error - pid->prevError) / dt;
    double dTerm = pid->kd * derivative;

    pid->prevError = error;

    return pTerm + iTerm + dTerm;
}

void pid_reset(PID_t* pid) {
    if (pid) {
        pid->intergral = 0.0;
        pid->prevError = 0.0;
        pid->prevTimeUS = 0; // Reset timing if your PID structure has this field
    }
}

void timer_task_callback_init(int periodUS, BlackBox_t* box, void (*cb)(void*)) {

    esp_timer_create_args_t config = {
        .callback = cb,  // Function to execute
        .dispatch_method = ESP_TIMER_TASK, // Where the function is called from
        .arg = box                         // Input argument
    };
    
    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&config, &timerHandle);
    esp_timer_start_periodic(timerHandle, periodUS);
}

void battery_callback(void* args) {
    
    BlackBox_t* box = (BlackBox_t*) args;
    if (!box) {
        return;
    }

    if (!mcpxQueue) {
        return;
    }

    uint16_t battery;
    if (xQueueReceive(mcpxQueue, &battery, pdMS_TO_TICKS(MCPx_DELAY)) == pdTRUE) {
        box->battery = battery;
        // ESP_LOGI(TAG, "Battery: %d", box->battery);
    }
}

void remote_data_callback(void* args) {
    BlackBox_t* box = (BlackBox_t*) args;

    if (!box) {
        return;
    }

    // Only send telemetry when we have a valid, active link
    if (!box->connectedToRemote || box->failsafeActive) {
        return;
    }

    int16_t package[RADIO_PAYLOAD_WIDTH / 2]; // 16 words

    // Angles: pitch, roll, yaw     = 3
    // Rates: pitch, roll, yaw      = 3
    // Flight mode                  = 1
    // PID outputs pitch, roll, yaw = 3
    // Motor periods A, B, C, D     = 4
    // Battery percentage           = 1
    // Total                       = 15

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
    package[14] = box->battery;

    xQueueSendToBack(radioTransmitterQueue, package, 0);
}
