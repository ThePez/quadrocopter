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
#include "imu.h"
#include "mcp3208.h"
#include "motors.h"

#define TAG "DRONE"

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t inputTaskhandle = NULL;
TaskHandle_t pidTaskHandle = NULL;

// Set default Scalers for the PID structs
static PIDParameters_t ratePid;
static PIDParameters_t rateZPid;
static PIDParameters_t anglePid;

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

// Wifi
static uint16_t* wifiPayload = NULL;

///////////////////////////////// Prototypes /////////////////////////////////

static void memory_init(void);

static void init_pid_params(PIDParameters_t* params, double kp, double kd, double ki);
static double pid_update(PIDParameters_t* params, PIDResult_t* values, double ref, double actual);
static void pid_reset(PIDResult_t* pid);
static void pid_timer_init(void);
static void pid_callback(void* args);
static void handle_pid_update(pid_config_packet_t* packet);

static void timer_task_callback_init(int periodUS, void (*cb)(void*));
static void battery_callback(void* args);
static void remote_data_callback(void* args);

//////////////////////////////////////////////////////////////////////////////

void app_main(void) {

    // Set Co-efficients for the PID control loops
    init_pid_params(&ratePid, 0.175, 0, 0);
    init_pid_params(&rateZPid, 0.1, 0, 0);
    init_pid_params(&anglePid, 1, 0, 0);

    // Setup PWM
    if (esc_pwm_init() != ESP_OK) {
        esp_restart();
    }

    // // SPI Bus Setup for radio and mcp3208
    // if (spi_bus_setup(HSPI_HOST) != ESP_OK) {
    //     esp_restart();
    // }

    // If a failure occurs in the call, the ESP will reset
    memory_init();

    // Start the C++ task
    imu_init();

    // Wait for the IMU initialisation to finish
    vTaskDelay(pdMS_TO_TICKS(200));

    uint8_t* macs[] = {remote_mac, bridge_mac};

    // Setup ESP-NOW (pair the remote here)
    esp_now_module_init(macs, 2);

    // // Register bridge as additional peer (used to tune the PID co-efficients)
    // esp_now_peer_info_t bridge_peer = {.channel = 0, .ifidx = WIFI_IF_STA, .encrypt = true};
    // memcpy(bridge_peer.peer_addr, bridge_mac, ESP_NOW_ETH_ALEN);
    // memcpy(bridge_peer.lmk, lmk_key, 16); // Same encryption keys
    // if (esp_now_add_peer(&bridge_peer) == ESP_OK) {
    //     ESP_LOGI(TAG, "Bridge added as peer");
    // } else {
    //     ESP_LOGE(TAG, "Bridge not paired");
    // }

    // Wait to ensure the wifiQueue is created for inter-task coms
    while (!wifiQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Initialise a callback for returning info back to the remote
    timer_task_callback_init(100000, remote_data_callback); // Interval of 100ms

    // Initialise a callback for check battery voltage
    // timer_task_callback_init(1000000, battery_callback);    // Interval of 1 second

    /* Create 2 control Tasks */
    xTaskCreatePinnedToCore(&pid_control, "PID_Task", SYS_STACK, NULL, SYS_PRIO + 2, &pidTaskHandle, (BaseType_t) 1);
    xTaskCreatePinnedToCore(&input_control, "INPUT_TASK", SYS_STACK, NULL, SYS_PRIO, &inputTaskhandle, (BaseType_t) 0);

    // Small delay to ensure all setup is complete before the clock is started
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialise the timer for PID Task signalling
    pid_timer_init();
}

/* Control Tasks */

void input_control(void* pvParams) {

    double inputReading;

    // Wait for the radio queue to be initialised (should already be done by this point)
    while (!wifiQueue) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Input Task Setup");

    while (1) {
        if (xQueueReceive(wifiQueue, wifiPayload, portMAX_DELAY) == pdTRUE) {

            uint16_t cmd = wifiPayload[0];
            switch (cmd) {
            case REMOTE_UPDATE:
                droneData->lastRemoteTime = esp_timer_get_time();

                // Only set armed state if not in angle failsafe state
                if (!droneData->angle_failsafe_active) {
                    droneData->armed = 1;
                }

                inputReading = mapf(wifiPayload[1], 0, ADC_MAX, MIN_THROTTLE, MAX_THROTTLE);
                remoteIn->throttle = (inputReading < MIN_THROTTLE + 20) ? MIN_THROTTLE : inputReading;
                // Pitch Input
                inputReading = mapf(wifiPayload[2], 0, ADC_MAX, -MAX_RATE, MAX_RATE);
                remoteIn->pitch = (fabs(inputReading) < 5) ? 0 : inputReading;
                // Roll Input
                inputReading = mapf(wifiPayload[3], 0, ADC_MAX, -MAX_RATE, MAX_RATE);
                remoteIn->roll = (fabs(inputReading) < 5) ? 0 : inputReading;
                // Yaw Input (inverted due to physical position of joystick on remote)
                inputReading = -mapf(wifiPayload[4], 0, ADC_MAX, -MAX_RATE, MAX_RATE);
                remoteIn->yaw = (fabs(inputReading) < 5) ? 0 : inputReading;
                // Flight mode
                set_flight_mode((wifiPayload[5] == ACRO) ? ACRO : STABILISE);
                break;

            case PID_UPDATE:
                pid_config_packet_t* pidPacket = (pid_config_packet_t*) wifiPayload;
                ESP_LOGI(TAG, "PID Coefficient update");
                handle_pid_update(pidPacket);
                break;

            default:
                ESP_LOGW(TAG, "Unknown cmd ID %d", cmd);
                break;
            }
        }
    }
}

void pid_control(void* pvParams) {

    ESP_LOGI(TAG, "PID Task Setup");

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t now = esp_timer_get_time();

        // Check drone for critical angles
        if (fabs(imuData->pitchAngle) > FAIL_ANGLE || fabs(imuData->rollAngle) > FAIL_ANGLE) {
            angle_failsafe();
            continue;
        } else if (droneData->angle_failsafe_active) {
            // To reset drone, throttle must be off and drone must be level
            bool throttle_safe = remoteIn->throttle < (MIN_THROTTLE + 50);
            bool level = (fabs(imuData->pitchAngle) < 5) && (fabs(imuData->rollAngle) < 5);
            if (throttle_safe && level) {
                ESP_LOGI(TAG, "Angle Failsafe cleared");
                droneData->angle_failsafe_active = 0;
            } else {
                motor_shutdown();
                continue;
            }
        }

        // Check last connection to remote
        if (now - droneData->lastRemoteTime > FAILSAFE_TIMEOUT_US) {
            comms_failsafe();
            continue;
        } else if (droneData->comms_failsafe_active) {
            // Comms recovered - clear failsafe
            droneData->comms_failsafe_active = 0;
            ESP_LOGI(TAG, "Comms failsafe cleared - link restored");
        }

        // Only run control if armed and no failsafes active
        if (!droneData->armed) {
            motor_shutdown();
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

/* Memory setup */

static void memory_init(void) {
    wifiPayload = pvPortMalloc(sizeof(uint16_t) * 16);
    pidRate = pvPortMalloc(sizeof(PIDFeedback_t));
    pidAngle = pvPortMalloc(sizeof(PIDFeedback_t));
    remoteIn = pvPortMalloc(sizeof(TargetParameters_t));
    motors = pvPortMalloc(sizeof(PWM_t));
    outputPID = pvPortMalloc(sizeof(PIDFinal_t));
    droneData = pvPortMalloc(sizeof(DroneConfig_t));

    // If any malloc's failed restart the MSU

    if (!wifiPayload || !pidRate || !pidAngle || !remoteIn || !motors || !outputPID || !droneData) {
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

    // Rate PID's
    pid_reset(pidRate->pitch);
    pid_reset(pidRate->roll);
    pid_reset(pidRate->yaw);

    // Angle PID's
    pid_reset(pidAngle->pitch);
    pid_reset(pidAngle->roll);

    // Drone info
    droneData->armed = 0;
    droneData->lastRemoteTime = 0;
    droneData->angle_failsafe_active = 0;
    droneData->comms_failsafe_active = 0;
    droneData->mode = ACRO;
}

/* Saftey related functions */

void set_flight_mode(FlightMode_t mode) {
    if (mode == droneData->mode) {
        return;
    }

    // Update flight mode
    droneData->mode = mode;

    // Reset PID integrators when switching modes to prevent windup

    // Rates
    pid_reset(pidRate->pitch);
    pid_reset(pidRate->roll);
    pid_reset(pidRate->yaw);
    // Angles
    pid_reset(pidAngle->pitch);
    pid_reset(pidAngle->roll);

    ESP_LOGI(TAG, "Control mode: %s", mode == STABILISE ? "Stabilise" : "Acro");
}

void motor_shutdown(void) {
    for (uint8_t i = 0; i < 4; i++) {
        esc_pwm_set_duty_cycle((MotorIndex) i, MIN_THROTTLE);
    }

    motors->motorA = MIN_THROTTLE;
    motors->motorB = MIN_THROTTLE;
    motors->motorC = MIN_THROTTLE;
    motors->motorD = MIN_THROTTLE;
}

void angle_failsafe(void) {
    if (droneData->angle_failsafe_active) {
        return;
    }

    droneData->angle_failsafe_active = 1;
    droneData->armed = 0;
    ESP_LOGE(TAG, "FAILSAFE: angles Pitch %.1f, Roll %.1f", imuData->pitchAngle, imuData->rollAngle);
    ESP_LOGW(TAG, "FAILSAFE: Level the drone and throttle down to reset");
    // Kill the motors
    motor_shutdown();
}

void comms_failsafe(void) {
    if (droneData->comms_failsafe_active || !droneData->armed) {
        return;
    }

    droneData->comms_failsafe_active = 1;
    droneData->armed = 0;
    ESP_LOGE(TAG, "FAILSAFE: No wifi link for at least 1 second");
    ESP_LOGW(TAG, "FAILSAFE: May need to reset both remote and drone to reset");
    // Kill the motors
    motor_shutdown();
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

/* PID control loop related functions */

static void init_pid_params(PIDParameters_t* params, double kp, double kd, double ki) {
    params->divLimit = PID_DIV_LIMIT;
    params->intLimit = PID_INT_LIMIT;
    params->dt = PID_LOOP_FREQ;
    params->kp = kp;
    params->kd = kd;
    params->ki = ki;
}

static double pid_update(PIDParameters_t* params, PIDResult_t* values, double ref, double actual) {

    double dt = params->dt * 1e-6;

    double error = ref - actual;
    // P
    values->proportional = params->kp * error;
    // I
    values->intergral += params->ki * (error) *dt;
    values->intergral = constrainf(values->intergral, -params->intLimit, params->intLimit);
    // D
    values->derivative = params->kd * (error - values->prevError) / dt;
    // Update Error
    values->prevError = error;

    return values->proportional + values->intergral + values->derivative;
}

static void pid_reset(PIDResult_t* pid) {
    if (pid) {
        pid->proportional = 0.0;
        pid->intergral = 0.0;
        pid->derivative = 0.0;
        pid->prevError = 0.0;
    }
}

static void pid_timer_init(void) {
    const esp_timer_create_args_t args = {.callback = pid_callback, .dispatch_method = ESP_TIMER_ISR};
    esp_timer_handle_t pid_timer = NULL;
    esp_timer_create(&args, &pid_timer);
    esp_timer_start_periodic(pid_timer, 2500); // us
    ESP_LOGI(TAG, "PID TIMER Setup");
}

static void pid_callback(void* args) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pidTaskHandle) {
        vTaskNotifyGiveFromISR(pidTaskHandle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void handle_pid_update(pid_config_packet_t* packet) {

    const char* axis_names[] = {"Pitch & Roll", "Yaw"};
    const char* mode_names[] = {"Rate", "Angle"};

    if (packet->axis > 1) {
        ESP_LOGE(TAG, "Invalid axis: %d", packet->axis);
        return;
    }

    if (packet->mode > 1) {
        ESP_LOGE(TAG, "Invalid mode: %d", packet->mode);
        return;
    }

    // Yaw only supports Rate mode
    if (packet->axis == 1 && packet->mode != 0) {
        ESP_LOGW(TAG, "Yaw axis only supports Rate mode, ignoring Angle request");
        packet->mode = 0;
    }

    ESP_LOGI(TAG, "PID Update - %s %s: Kp=%.4f Ki=%.4f Kd=%.4f", axis_names[packet->axis], mode_names[packet->mode],
             packet->kp, packet->ki, packet->kd);

    // Update the appropriate PID controller
    if (!packet->mode) {
        // Rate mode
        PIDParameters_t* pid = (packet->axis == 1) ? &rateZPid : &ratePid;
        pid->kp = packet->kp;
        pid->ki = packet->ki;
        pid->kd = packet->kd;

        // Reset integrator when changing gains
        if (!packet->axis) {
            pid_reset(pidRate->pitch);
            pid_reset(pidRate->roll);
        } else {
            pid_reset(pidRate->yaw);
        }

    } else {
        // Angle mode (only Pitch and Roll)
        if (!packet->axis) {
            anglePid.kp = packet->kp;
            anglePid.ki = packet->ki;
            anglePid.kd = packet->kd;
            // Reset integrators
            pid_reset(pidAngle->pitch);
            pid_reset(pidAngle->roll);
        }
    }

    ESP_LOGI(TAG, "PID coefficients updated successfully");
}

/* Functions run in the ESP-TIMER-TASK */

static void timer_task_callback_init(int periodUS, void (*cb)(void*)) {

    esp_timer_create_args_t config = {
        .callback = cb,                    // Function to execute
        .dispatch_method = ESP_TIMER_TASK, // Where the function is called from
        .arg = NULL                        // Input argument
    };

    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&config, &timerHandle);
    esp_timer_start_periodic(timerHandle, periodUS);
}

static void battery_callback(void* args) {
    if (!mcpxQueue) {
        return;
    }

    uint16_t battery;
    if (xQueueReceive(mcpxQueue, &battery, pdMS_TO_TICKS(MCPx_DELAY)) == pdTRUE) {
        droneData->battery = mapf(battery, 0, 1000, 0, ADC_MAX);
        ESP_LOGI(TAG, "Battery: %d", droneData->battery);
    }
}

static void remote_data_callback(void* args) {
    int16_t package[16]; // 16 words / 32 bytes
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
    package[14] = 0; // MCP3208 not on drone atm -> force to 0

    // ID for Bridge Telemetry data
    package[15] = 3;

    esp_err_t result = esp_now_send((uint8_t*) bridge_mac, (uint8_t*) package, 32);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "data callback send failed");
    }
}
