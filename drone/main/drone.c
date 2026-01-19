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
#include "espnow_comm.h"
#include "mcp3208.h"
#include "motors.h"

#define TAG "DRONE"
#define FAILSAFE_TIMEOUT_US 1500000 // 1.5 second
#define PID_LOOP_FREQ 2500          // 400 Hz -> 2.5ms -> 2500us
#define PID_INT_LIMIT 400

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t inputTaskhandle = NULL;
TaskHandle_t pidTaskHandle = NULL;

// Set default Scalers for the PID structs
static PIDParameters_t ratePid = {.kp = 0.7, .ki = 0.00, .kd = 0.00, .intLimit = PID_INT_LIMIT, .dt = PID_LOOP_FREQ};
static PIDParameters_t rateZPid = {.kp = 0.2, .ki = 0.00, .kd = 0.00, .intLimit = PID_INT_LIMIT, .dt = PID_LOOP_FREQ};
static PIDParameters_t anglePid = {.kp = 0.4, .ki = 0.00, .kd = 0.00, .intLimit = PID_INT_LIMIT, .dt = PID_LOOP_FREQ};

// Various configs for general operation
static DroneConfig_t* droneData = NULL;

// 5 control PID loops containers
static PIDFeedback_t* pidRate = NULL;
static PIDFeedback_t* pidAngle = NULL;

static PIDFinal_t* outputPID = NULL;

// Remote set point into
static TargetParameters_t* remoteIn = NULL;

// PWM duty cycles
static PWM_t* motors = NULL;

// Radio
static uint16_t* radioPayload = NULL;

//////////////////////////////////////////////////////////////////////////////

void app_main(void) {

    // Setup PWM
    if (esc_pwm_init() != ESP_OK) {
        esp_restart();
    }

    // SPI Bus Setup for radio and mcp3208
    if (spi_bus_setup(HSPI_HOST) != ESP_OK) {
        esp_restart();
    }

    // If a failure occurs in the call, the ESP will reset
    memory_init();

    // Start the C++ task
    imu_init();

    // Setup ESP-NOW
    esp_now_module_init(remote_mac);

    // Wait until both input queues are created
    while (!wifiQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Initialise a callback task for returning info back to the remote
    timer_task_callback_init(200000, remote_data_callback); // Interval of 200ms
    // timer_task_callback_init(1000000, battery_callback);    // Interval of 1 second

    /* Create 2 control Tasks */
    xTaskCreatePinnedToCore(&pid_control, "PID_Task", SYS_STACK, NULL, SYS_PRIO + 2, &pidTaskHandle, (BaseType_t) 1);
    xTaskCreatePinnedToCore(&input_control, "INPUT_TASK", SYS_STACK, NULL, SYS_PRIO, &inputTaskhandle, (BaseType_t) 0);

    vTaskDelay(pdMS_TO_TICKS(10));

    // Initialise the timer for PID Task signalling
    pid_timer_init();
}

void input_control(void* pvParams) {

    double inputReading;

    // Wait for the radio queue to be initialised
    while (!wifiQueue) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Input Task Setup");

    while (1) {
        if (xQueueReceive(wifiQueue, radioPayload, portMAX_DELAY) == pdTRUE) {
            droneData->lastRemoteTime = esp_timer_get_time();
            droneData->armed = 1;
            inputReading = mapf(radioPayload[1], 0, ADC_MAX, MIN_THROTTLE, MAX_THROTTLE);
            remoteIn->throttle = (inputReading < MIN_THROTTLE + 20) ? MIN_THROTTLE : inputReading;
            // Pitch Input
            inputReading = -mapf(radioPayload[2], 0, ADC_MAX, -MAX_RATE, MAX_RATE);
            remoteIn->pitch = (fabs(inputReading) < 5) ? 0 : inputReading;
            // Roll Input
            inputReading = -mapf(radioPayload[3], 0, ADC_MAX, -MAX_RATE, MAX_RATE);
            remoteIn->roll = (fabs(inputReading) < 5) ? 0 : inputReading;
            // Yaw Input
            inputReading = mapf(radioPayload[4], 0, ADC_MAX, -MAX_RATE, MAX_RATE);
            remoteIn->yaw = (fabs(inputReading) < 5) ? 0 : inputReading;
            // Flight mode
            set_flight_mode((radioPayload[5] == ACRO) ? ACRO : STABILISE);
        }
    }
}

void pid_control(void* pvParams) {

    ESP_LOGI(TAG, "PID Task Setup");

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t now = esp_timer_get_time();

        // Check drone angle
        if (fabs(imuData->pitchAngle) > 50 || fabs(imuData->rollAngle) > 50) {
            angle_failsafe();
            continue;
        }

        // Check last connection to remote
        if (now - droneData->lastRemoteTime > FAILSAFE_TIMEOUT_US) {
            comms_failsafe();
            continue;
        }

        double pitchRateSetpoint, rollRateSetpoint, yawRateSetpoint;

        if (droneData->mode == STABILISE) {
            // ANGLE MODE: Cascaded control with outer angle loop

            // Map remote control inputs to angle setpoints
            double pitchAngleSetpoint = mapf(remoteIn->pitch, -MAX_RATE, MAX_RATE, -MAX_ANGLE, MAX_ANGLE);
            double rollAngleSetpoint = mapf(remoteIn->roll, -MAX_RATE, MAX_RATE, -MAX_ANGLE, MAX_ANGLE);

            pitchRateSetpoint = pid_update(&anglePid, pidAngle->pitch, pitchAngleSetpoint, imuData->pitchAngle);
            rollRateSetpoint = pid_update(&anglePid, pidAngle->roll, rollAngleSetpoint, imuData->rollAngle);

        } else {
            // RATE MODE: Direct rate control
            pitchRateSetpoint = remoteIn->pitch;
            rollRateSetpoint = remoteIn->roll;
        }

        // Yaw is still direct rate control in all control modes
        yawRateSetpoint = remoteIn->yaw;

        outputPID->pitch_pid = pid_update(&ratePid, pidRate->pitch, pitchRateSetpoint, imuData->pitchRate);
        outputPID->roll_pid = pid_update(&ratePid, pidRate->roll, rollRateSetpoint, imuData->rollRate);
        outputPID->yaw_pid = pid_update(&rateZPid, pidRate->yaw, yawRateSetpoint, imuData->yawRate);

        // Update ESCs
        update_escs();
    }
}

void memory_init(void) {
    radioPayload = pvPortMalloc(sizeof(uint16_t) * 16);
    pidRate = pvPortMalloc(sizeof(PIDFeedback_t));
    pidAngle = pvPortMalloc(sizeof(PIDFeedback_t));
    remoteIn = pvPortMalloc(sizeof(TargetParameters_t));
    motors = pvPortMalloc(sizeof(PWM_t));
    outputPID = pvPortMalloc(sizeof(PIDFinal_t));
    droneData = pvPortMalloc(sizeof(DroneConfig_t));

    // If any malloc's failed restart the MSU

    if (!radioPayload || !pidRate || !pidAngle || !remoteIn || !motors || !outputPID || !droneData) {
        esp_restart();
    }

    pidRate->pitch = pvPortMalloc(sizeof(PIDResult_t));
    pidRate->roll = pvPortMalloc(sizeof(PIDResult_t));
    pidRate->yaw = pvPortMalloc(sizeof(PIDResult_t));

    pidAngle->pitch = pvPortMalloc(sizeof(PIDResult_t));
    pidAngle->roll = pvPortMalloc(sizeof(PIDResult_t));

    if (!pidRate->pitch || !pidRate->roll || !pidRate->yaw || !pidAngle->pitch || !pidAngle->roll) {
        esp_restart();
    }

    pidRate->pitch->intergral = 0;
    pidRate->roll->intergral = 0;
    pidRate->yaw->intergral = 0;

    pidAngle->pitch->intergral = 0;
    pidAngle->roll->intergral = 0;
    pidAngle->yaw->intergral = 0;

    droneData->armed = 0;
    droneData->lastRemoteTime = 0;
    droneData->mode = ACRO;
}

void set_flight_mode(FlightMode_t mode) {
    if (mode != droneData->mode) {
        droneData->mode = mode;
        // Reset PID integrators when switching modes to prevent windup
        pid_reset(pidRate->pitch);
        pid_reset(pidRate->roll);
        pid_reset(pidRate->yaw);

        pid_reset(pidAngle->pitch);
        pid_reset(pidAngle->roll);

        ESP_LOGI(TAG, "Switched to %s mode", mode == STABILISE ? "ANGLE" : "RATE");
    }
}

void angle_failsafe(void) {
    if (!droneData->armed) {
        return;
    }

    droneData->armed = 0;
    // ESP_LOGW(TAG, "FAILSAFE: drone angle too steep");

    // Force motors to MIN_THROTTLE
    for (uint8_t i = 0; i < 4; i++) {
        esc_pwm_set_duty_cycle((MotorIndex) i, MIN_THROTTLE);
    }
}

void comms_failsafe(void) {
    if (!droneData->armed) {
        return;
    }

    // ESP_LOGE(TAG, "FAILSAFE TRIGGERED: No wifi link for at least 1.5 second");
    droneData->armed = 0;

    // Force motors to MIN_THROTTLE
    for (uint8_t i = 0; i < 4; i++) {
        esc_pwm_set_duty_cycle((MotorIndex) i, MIN_THROTTLE);
    }
}

void update_escs(void) {

    // Low throttle input (ie off)
    if (remoteIn->throttle < 1050) {
        motors->motorA = MIN_THROTTLE;
        motors->motorB = MIN_THROTTLE;
        motors->motorC = MIN_THROTTLE;
        motors->motorD = MIN_THROTTLE;
    } else {
        double throttle = remoteIn->throttle;
        // Front left
        double speed = throttle - outputPID->pitch_pid + outputPID->roll_pid - outputPID->yaw_pid;
        motors->motorA = constrainf(speed, MIN_THROTTLE + 50, MAX_THROTTLE);

        // Rear left
        speed = throttle + outputPID->pitch_pid + outputPID->roll_pid + outputPID->yaw_pid;
        motors->motorB = constrainf(speed, MIN_THROTTLE + 50, MAX_THROTTLE);

        // Rear right
        speed = throttle + outputPID->pitch_pid - outputPID->roll_pid - outputPID->yaw_pid;
        motors->motorC = constrainf(speed, MIN_THROTTLE + 50, MAX_THROTTLE);

        // Front right
        speed = throttle - outputPID->pitch_pid - outputPID->roll_pid + outputPID->yaw_pid;
        motors->motorD = constrainf(speed, MIN_THROTTLE + 50, MAX_THROTTLE);
    }

    esc_pwm_set_duty_cycle(MOTOR_A, motors->motorA);
    esc_pwm_set_duty_cycle(MOTOR_B, motors->motorB);
    esc_pwm_set_duty_cycle(MOTOR_C, motors->motorC);
    esc_pwm_set_duty_cycle(MOTOR_D, motors->motorD);
}

double pid_update(PIDParameters_t* params, PIDResult_t* values, double ref, double actual) {

    double dt = params->dt * 1e-6;

    double error = ref - actual;
    // P
    values->proportional = params->kp * error;
    // I
    values->intergral += params->ki * (error + values->prevError) * dt;
    values->intergral = constrainf(values->intergral, -params->intLimit, params->intLimit);
    // D
    values->derivative = params->kd * (error - values->prevError) / dt;
    // Update Error
    values->prevError = error;

    return values->proportional + values->intergral + values->derivative;
}

void pid_reset(PIDResult_t* pid) {
    if (pid) {
        pid->proportional = 0.0;
        pid->intergral = 0.0;
        pid->derivative = 0.0;
        pid->prevError = 0.0;
    }
}

void timer_task_callback_init(int periodUS, void (*cb)(void*)) {

    esp_timer_create_args_t config = {
        .callback = cb,                    // Function to execute
        .dispatch_method = ESP_TIMER_TASK, // Where the function is called from
        .arg = NULL                        // Input argument
    };

    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&config, &timerHandle);
    esp_timer_start_periodic(timerHandle, periodUS);
}

void pid_timer_init(void) {
    const esp_timer_create_args_t args = {.callback = pid_callback, .dispatch_method = ESP_TIMER_ISR};
    esp_timer_handle_t pid_timer = NULL;
    esp_timer_create(&args, &pid_timer);
    esp_timer_start_periodic(pid_timer, 2500); // us
    ESP_LOGI(TAG, "PID TIMER Setup");
}

void pid_callback(void* args) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pidTaskHandle) {
        vTaskNotifyGiveFromISR(pidTaskHandle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void battery_callback(void* args) {
    if (!mcpxQueue) {
        return;
    }

    uint16_t battery;
    if (xQueueReceive(mcpxQueue, &battery, pdMS_TO_TICKS(MCPx_DELAY)) == pdTRUE) {
        droneData->battery = mapf(battery, 0, 1000, 0, ADC_MAX);
        ESP_LOGI(TAG, "Battery: %d", droneData->battery);
    }
}

void remote_data_callback(void* args) {
    int16_t package[16]; // 16 words

    // Angles: pitch, roll, yaw     = 3
    // Rates: pitch, roll, yaw      = 3
    // Flight mode                  = 1
    // PID outputs pitch, roll, yaw = 3
    // Motor periods A, B, C, D     = 4
    // Battery percentage           = 1
    // Total                       = 15

    // Angles
    package[0] = (int16_t) imuData->pitchAngle;
    package[1] = (int16_t) imuData->rollAngle;
    package[2] = (int16_t) imuData->yawAngle;
    // Rate
    package[3] = (int16_t) imuData->pitchRate;
    package[4] = (int16_t) imuData->rollRate;
    package[5] = (int16_t) imuData->yawRate;
    // Flight mode
    package[6] = (int16_t) droneData->mode;
    // PID outputs
    package[7] = (int16_t) outputPID->pitch_pid;
    package[8] = (int16_t) outputPID->roll_pid;
    package[9] = (int16_t) outputPID->yaw_pid;
    // Motor periods
    package[10] = motors->motorA;
    package[11] = motors->motorB;
    package[12] = motors->motorC;
    package[13] = motors->motorD;
    // Battery Voltage
    package[14] = droneData->battery;

    esp_err_t result = esp_send_packet(&package, 32);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "data callback send failed");
    }
}
