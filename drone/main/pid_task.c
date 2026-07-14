/*
 *****************************************************************************
 * File: pid_task.c
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#include "pid_task.h"

#include "common_functions.h"
#include "device_config.h"
#include "drone.h"
#include "imu.h"
#include "kalman.h"
#include "motors.h"
#include "pid.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>
#include <stdbool.h>

#define TAG "PID_TASK"

#define PID_TASK_STACK (configMINIMAL_STACK_SIZE * 2)
#define PID_TASK_PRIO (tskIDLE_PRIORITY + 4)

struct pid_feedback_t {
    struct pid_result_t pitch;
    struct pid_result_t roll;
    struct pid_result_t yaw;
};

static TaskHandle_t pidTaskHandle = NULL;

// Co-efficients for the PID control loops
static struct pid_parameters_t ratePid;
static struct pid_parameters_t rateZPid;
static struct pid_parameters_t anglePid;

// Running P/I/D state for each of the 5 control loops
static struct pid_feedback_t pidRate;
static struct pid_feedback_t pidAngle;

struct pid_final_t outputPID;
struct pwm_t motors;

// Drives all 4 ESCs to minimum throttle and zeroes the reported duty cycles.
static void motor_shutdown(void) {
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    uint16_t minThrottle = (uint16_t) droneCfg.min_throttle;
    xSemaphoreGive(cfgMutex);

    for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        esc_pwm_set_duty_cycle((MotorIndex) i, minThrottle);
    }

    motors.motorA = minThrottle;
    motors.motorB = minThrottle;
    motors.motorC = minThrottle;
    motors.motorD = minThrottle;
}

// Latches the angle failsafe (disarms and kills the motors) once pitch/roll
// exceeds droneCfg.fail_angle. No-op if the failsafe is already active.
static void angle_failsafe(struct imu_packet_t* attitude) {
    xSemaphoreTake(droneData.mutex, portMAX_DELAY);
    if (droneData.status_mask & ANGLE_FAIL) {
        xSemaphoreGive(droneData.mutex);
        return;
    }

    droneData.status_mask |= ANGLE_FAIL;
    droneData.armed = 0;
    xSemaphoreGive(droneData.mutex);

    ESP_LOGE(TAG, "FAILSAFE: angles Pitch %.1f, Roll %.1f", attitude->pitchAngle,
             attitude->rollAngle);
    ESP_LOGW(TAG, "FAILSAFE: Level the drone and throttle down to reset");
    // Kill the motors
    motor_shutdown();
}

// Latches the comms failsafe (disarms and kills the motors) once the link to
// the remote has been silent for longer than droneCfg.coms_timeout_us.
static void comms_failsafe(void) {
    xSemaphoreTake(droneData.mutex, portMAX_DELAY);
    if ((droneData.status_mask & COMS_FAIL) || !droneData.armed) {
        xSemaphoreGive(droneData.mutex);
        return;
    }

    droneData.status_mask |= COMS_FAIL;
    droneData.armed = 0;
    xSemaphoreGive(droneData.mutex);

    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    int64_t comsTimeoutUs = droneCfg.coms_timeout_us;
    xSemaphoreGive(cfgMutex);

    ESP_LOGE(TAG, "FAILSAFE: No wifi link for %lld second", (comsTimeoutUs / 1000000));
    ESP_LOGW(TAG, "FAILSAFE: May need to reset both remote and drone to reset");
    // Kill the motors
    motor_shutdown();
}

// Mixes throttle with the latest PID outputs into each motor's duty cycle
// (or forces minimum throttle if the stick is low or the battery is critical)
// and writes the results to the ESCs.
static void update_escs(void) {
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    double min = droneCfg.min_throttle;
    double max = droneCfg.max_throttle;
    uint16_t criticalVoltage = droneCfg.critical_voltage;
    xSemaphoreGive(cfgMutex);

    double idle = min + 50;
    double throttle = remoteIn.throttle;

    // Low throttle input (ie off)
    if (throttle < idle || droneData.battery < criticalVoltage) {
        motors.motorA = min;
        motors.motorB = min;
        motors.motorC = min;
        motors.motorD = min;
    } else {
        
        double pitch = outputPID.pitch_pid;
        double roll = outputPID.roll_pid;
        double yaw = outputPID.yaw_pid;
        // Front left
        double speed = throttle - pitch + roll - yaw;
        motors.motorA = constrainf(speed, idle, max);

        // Rear left
        speed = throttle + pitch + roll + yaw;
        motors.motorB = constrainf(speed, idle, max);

        // Rear right
        speed = throttle + pitch - roll - yaw;
        motors.motorC = constrainf(speed, idle, max);

        // Front right
        speed = throttle - pitch - roll + yaw;
        motors.motorD = constrainf(speed, idle, max);
    }

    esc_pwm_set_duty_cycle(MOTOR_A, motors.motorA);
    esc_pwm_set_duty_cycle(MOTOR_B, motors.motorB);
    esc_pwm_set_duty_cycle(MOTOR_C, motors.motorC);
    esc_pwm_set_duty_cycle(MOTOR_D, motors.motorD);
}

void set_flight_mode(enum flight_mode_t mode) {
    xSemaphoreTake(droneData.mutex, portMAX_DELAY);
    if (mode == droneData.mode) {
        xSemaphoreGive(droneData.mutex);
        return;
    }

    // Update flight mode
    droneData.mode = mode;
    xSemaphoreGive(droneData.mutex);

    // Reset PID integrators when switching modes to prevent windup

    // Rates
    pid_reset(&pidRate.pitch);
    pid_reset(&pidRate.roll);
    pid_reset(&pidRate.yaw);
    // Angles
    pid_reset(&pidAngle.pitch);
    pid_reset(&pidAngle.roll);

    ESP_LOGI(TAG, "Control mode: %s", mode == STABILISE ? "Stabilise" : "Acro");
}

void pid_handle_config_update(struct pid_config_telemetry_t* packet) {

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

    ESP_LOGI(TAG, "PID Update - %s %s: Kp=%.4f Ki=%.4f Kd=%.4f", axis_names[packet->axis],
             mode_names[packet->mode], packet->kp, packet->ki, packet->kd);

    // Update the appropriate PID controller
    if (!packet->mode) {
        // Rate mode
        struct pid_parameters_t* pid = (packet->axis == 1) ? &rateZPid : &ratePid;
        pid->kp = packet->kp;
        pid->ki = packet->ki;
        pid->kd = packet->kd;

        // Reset integrator when changing gains
        if (!packet->axis) {
            pid_reset(&pidRate.pitch);
            pid_reset(&pidRate.roll);
        } else {
            pid_reset(&pidRate.yaw);
        }

    } else {
        // Angle mode (only Pitch and Roll)
        if (!packet->axis) {
            anglePid.kp = packet->kp;
            anglePid.ki = packet->ki;
            anglePid.kd = packet->kd;
            // Reset integrators
            pid_reset(&pidAngle.pitch);
            pid_reset(&pidAngle.roll);
        }
    }

    ESP_LOGI(TAG, "PID coefficients updated successfully");
}

// ISR: wakes pid_control() for its next control cycle.
static IRAM_ATTR void pid_timer_callback(void* args) {
    ARG_UNUSED(args);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pidTaskHandle) {
        vTaskNotifyGiveFromISR(pidTaskHandle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Creates and starts the ISR timer that ticks the PID loop at PID_LOOP_FREQ.
static esp_err_t pid_timer_init(void) {
    const esp_timer_create_args_t args = {.callback = pid_timer_callback,
                                          .dispatch_method = ESP_TIMER_ISR};
    esp_timer_handle_t pid_timer = NULL;
    CHECK_ERR(esp_timer_create(&args, &pid_timer), "PID timer create failed");
    CHECK_ERR(esp_timer_start_periodic(pid_timer, PID_LOOP_FREQ), "PID timer start failed");
    ESP_LOGI(TAG, "PID Timer Initialised");
    return ESP_OK;
}

// Main PID task loop: on each timer tick, updates the attitude estimate,
// checks failsafes, runs the cascaded rate/angle PID controllers for the
// current flight mode, and pushes the results to the ESCs.
static void pid_control(void* pvParams) {
    ARG_UNUSED(pvParams);

    // Set Co-efficients for the PID control loops from the loaded config
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    pid_init_params(&ratePid, droneCfg.rate_pitch_roll.kp, droneCfg.rate_pitch_roll.kd,
                    droneCfg.rate_pitch_roll.ki, droneCfg.rate_pitch_roll.dtermAlpha);
    pid_init_params(&rateZPid, droneCfg.rate_yaw.kp, droneCfg.rate_yaw.kd, droneCfg.rate_yaw.ki,
                    droneCfg.rate_yaw.dtermAlpha);
    pid_init_params(&anglePid, droneCfg.angle_pitch_roll.kp, droneCfg.angle_pitch_roll.kd,
                    droneCfg.angle_pitch_roll.ki, droneCfg.angle_pitch_roll.dtermAlpha);
    xSemaphoreGive(cfgMutex);

    ESP_LOGI(TAG, "PID Task Setup");

    struct imu_packet_t kalman;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t now = esp_timer_get_time();

        // Non-blocking: if no new IMU sample has arrived this cycle, kalman_get()
        // below keeps returning the last known estimate rather than stalling.
        struct imu_packet_t imuSample;
        if (xQueueReceive(imuQueue, &imuSample, 0) == pdTRUE) {
            kalman_update(&imuSample);
        }

        kalman_get(&kalman);

        xSemaphoreTake(cfgMutex, portMAX_DELAY);
        double failAngle = droneCfg.fail_angle;
        xSemaphoreGive(cfgMutex);

        // Check drone for critical angles
        if (fabs(kalman.pitchAngle) > failAngle || fabs(kalman.rollAngle) > failAngle) {
            angle_failsafe(&kalman);
            continue;
        }

        // Snapshot the shared fields once per iteration under the lock, then make
        // decisions off the local copies so the mutex isn't held for the whole loop body
        xSemaphoreTake(droneData.mutex, portMAX_DELAY);
        uint8_t statusMask = droneData.status_mask;
        int64_t lastRemoteTime = droneData.lastRemoteTime;
        uint8_t armed = droneData.armed;
        enum flight_mode_t mode = droneData.mode;
        xSemaphoreGive(droneData.mutex);

        xSemaphoreTake(cfgMutex, portMAX_DELAY);
        double minThrottle = droneCfg.min_throttle;
        double maxRate = droneCfg.max_rate;
        double maxAngle = droneCfg.max_angle;
        int64_t comsTimeoutUs = droneCfg.coms_timeout_us;
        xSemaphoreGive(cfgMutex);

        if (statusMask & ANGLE_FAIL) {
            // To reset drone, throttle must be off and drone must be level
            bool throttle_safe = remoteIn.throttle < (minThrottle + 50);
            bool level = (fabs(kalman.pitchAngle) < 5) && (fabs(kalman.rollAngle) < 5);
            if (throttle_safe && level) {
                ESP_LOGI(TAG, "Angle Failsafe cleared");
                xSemaphoreTake(droneData.mutex, portMAX_DELAY);
                droneData.status_mask &= ~ANGLE_FAIL;
                xSemaphoreGive(droneData.mutex);
            } else {
                motor_shutdown();
                continue;
            }
        }

        // Check last connection to remote
        if (now - lastRemoteTime > comsTimeoutUs) {
            comms_failsafe();
            continue;
        } else if (statusMask & COMS_FAIL) {
            // Comms recovered - clear failsafe
            xSemaphoreTake(droneData.mutex, portMAX_DELAY);
            droneData.status_mask &= ~COMS_FAIL;
            xSemaphoreGive(droneData.mutex);
            ESP_LOGI(TAG, "Comms failsafe cleared - link restored");
        }

        // Only run control if armed and no failsafes active
        if (!armed) {
            motor_shutdown();
            continue;
        }

        double pitchRateSetpoint, rollRateSetpoint, yawRateSetpoint;

        if (mode == STABILISE) {
            // ANGLE MODE: Cascaded control with outer angle loop

            // Map remote control inputs to angle setpoints
            double pitchAngleSetpoint =
                mapf(remoteIn.pitch, -maxRate, maxRate, -maxAngle, maxAngle);
            double rollAngleSetpoint =
                mapf(remoteIn.roll, -maxRate, maxRate, -maxAngle, maxAngle);

            pitchRateSetpoint =
                pid_update(&anglePid, &pidAngle.pitch, pitchAngleSetpoint, kalman.pitchAngle);
            rollRateSetpoint =
                pid_update(&anglePid, &pidAngle.roll, rollAngleSetpoint, kalman.rollAngle);

        } else {
            // RATE MODE: Direct rate control
            pitchRateSetpoint = remoteIn.pitch;
            rollRateSetpoint = remoteIn.roll;
        }

        // Yaw is still direct rate control in all control modes
        yawRateSetpoint = remoteIn.yaw;

        outputPID.pitch_pid =
            pid_update(&ratePid, &pidRate.pitch, pitchRateSetpoint, kalman.pitchRate);
        outputPID.roll_pid = pid_update(&ratePid, &pidRate.roll, rollRateSetpoint, kalman.rollRate);
        outputPID.yaw_pid = pid_update(&rateZPid, &pidRate.yaw, yawRateSetpoint, kalman.yawRate);

        // Update ESCs
        update_escs();
    }
}

esp_err_t pid_task_init(void) {
    BaseType_t result = xTaskCreatePinnedToCore(pid_control, "PID_TASK", PID_TASK_STACK, NULL,
                                                PID_TASK_PRIO, &pidTaskHandle, (BaseType_t) 1);
    CHECK_ERR_NO_LOG(pid_timer_init());
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
