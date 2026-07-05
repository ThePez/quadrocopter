/*
 *****************************************************************************
 * File: pid_task.c
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#include "pid_task.h"

#include "common_functions.h"
#include "drone.h"
#include "imu.h"
#include "kalman.h"
#include "motors.h"
#include "pid.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
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

static void motor_shutdown(void) {
    for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        esc_pwm_set_duty_cycle((MotorIndex) i, MIN_THROTTLE);
    }

    motors.motorA = MIN_THROTTLE;
    motors.motorB = MIN_THROTTLE;
    motors.motorC = MIN_THROTTLE;
    motors.motorD = MIN_THROTTLE;
}

static void angle_failsafe(struct imu_packet_t* attitude) {
    if (droneData.status_mask & ANGLE_FAIL) {
        return;
    }

    droneData.status_mask |= ANGLE_FAIL;
    droneData.armed = 0;
    ESP_LOGE(TAG, "FAILSAFE: angles Pitch %.1f, Roll %.1f", attitude->pitchAngle,
             attitude->rollAngle);
    ESP_LOGW(TAG, "FAILSAFE: Level the drone and throttle down to reset");
    // Kill the motors
    motor_shutdown();
}

static void comms_failsafe(void) {
    if ((droneData.status_mask & COMS_FAIL) || !droneData.armed) {
        return;
    }

    droneData.status_mask |= COMS_FAIL;
    droneData.armed = 0;
    ESP_LOGE(TAG, "FAILSAFE: No wifi link for at least 1 second");
    ESP_LOGW(TAG, "FAILSAFE: May need to reset both remote and drone to reset");
    // Kill the motors
    motor_shutdown();
}

static void update_escs(void) {

    // Low throttle input (ie off)
    if (remoteIn.throttle < (MIN_THROTTLE + 50) || droneData.battery < CRITICAL_VOLTAGE) {
        motors.motorA = MIN_THROTTLE;
        motors.motorB = MIN_THROTTLE;
        motors.motorC = MIN_THROTTLE;
        motors.motorD = MIN_THROTTLE;
    } else {
        double throttle = remoteIn.throttle;
        // Front left
        double speed = throttle - outputPID.pitch_pid + outputPID.roll_pid - outputPID.yaw_pid;
        motors.motorA = constrainf(speed, MIN_THROTTLE + 50, MAX_THROTTLE);

        // Rear left
        speed = throttle + outputPID.pitch_pid + outputPID.roll_pid + outputPID.yaw_pid;
        motors.motorB = constrainf(speed, MIN_THROTTLE + 50, MAX_THROTTLE);

        // Rear right
        speed = throttle + outputPID.pitch_pid - outputPID.roll_pid - outputPID.yaw_pid;
        motors.motorC = constrainf(speed, MIN_THROTTLE + 50, MAX_THROTTLE);

        // Front right
        speed = throttle - outputPID.pitch_pid - outputPID.roll_pid + outputPID.yaw_pid;
        motors.motorD = constrainf(speed, MIN_THROTTLE + 50, MAX_THROTTLE);
    }

    esc_pwm_set_duty_cycle(MOTOR_A, motors.motorA);
    esc_pwm_set_duty_cycle(MOTOR_B, motors.motorB);
    esc_pwm_set_duty_cycle(MOTOR_C, motors.motorC);
    esc_pwm_set_duty_cycle(MOTOR_D, motors.motorD);
}

void set_flight_mode(enum flight_mode_t mode) {
    if (mode == droneData.mode) {
        return;
    }

    // Update flight mode
    droneData.mode = mode;

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

static IRAM_ATTR void pid_timer_callback(void* args) {
    ARG_UNUSED(args);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pidTaskHandle) {
        vTaskNotifyGiveFromISR(pidTaskHandle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static esp_err_t pid_timer_init(void) {
    const esp_timer_create_args_t args = {.callback = pid_timer_callback,
                                          .dispatch_method = ESP_TIMER_ISR};
    esp_timer_handle_t pid_timer = NULL;
    CHECK_ERR(esp_timer_create(&args, &pid_timer), "PID timer create failed");
    CHECK_ERR(esp_timer_start_periodic(pid_timer, PID_LOOP_FREQ), "PID timer start failed");
    ESP_LOGI(TAG, "PID Timer Initialised");
    return ESP_OK;
}

static void pid_control(void* pvParams) {
    ARG_UNUSED(pvParams);

    // Set Co-efficients for the PID control loops
    pid_init_params(&ratePid, 0.175, 0, 0); // kp, kd, ki
    pid_init_params(&rateZPid, 0.1, 0, 0);  // kp, kd, ki
    pid_init_params(&anglePid, 1, 0, 0);    // kp, kd, ki

    ESP_LOGI(TAG, "PID Task Setup");

    struct imu_packet_t kalman;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t now = esp_timer_get_time();

        // Non-blocking: if no new IMU sample has arrived this cycle, kalman_get()
        // below keeps returning the last known estimate rather than stalling.
        struct imu_packet_t imuSample;
        if (xQueueReceive(imuQueue, &imuSample, 0) == pdTRUE) {
            kalman_update(&imuSample);
        }
        kalman_get(&kalman);

        // Check drone for critical angles
        if (fabs(kalman.pitchAngle) > FAIL_ANGLE || fabs(kalman.rollAngle) > FAIL_ANGLE) {
            angle_failsafe(&kalman);
            continue;
        } else if (droneData.status_mask & ANGLE_FAIL) {
            // To reset drone, throttle must be off and drone must be level
            bool throttle_safe = remoteIn.throttle < (MIN_THROTTLE + 50);
            bool level = (fabs(kalman.pitchAngle) < 5) && (fabs(kalman.rollAngle) < 5);
            if (throttle_safe && level) {
                ESP_LOGI(TAG, "Angle Failsafe cleared");
                droneData.status_mask &= ~ANGLE_FAIL;
            } else {
                motor_shutdown();
                continue;
            }
        }

        // Check last connection to remote
        if (now - droneData.lastRemoteTime > FAILSAFE_TIMEOUT_US) {
            comms_failsafe();
            continue;
        } else if (droneData.status_mask & COMS_FAIL) {
            // Comms recovered - clear failsafe
            droneData.status_mask &= ~COMS_FAIL;
            ESP_LOGI(TAG, "Comms failsafe cleared - link restored");
        }

        // Only run control if armed and no failsafes active
        if (!droneData.armed) {
            motor_shutdown();
            continue;
        }

        double pitchRateSetpoint, rollRateSetpoint, yawRateSetpoint;

        if (droneData.mode == STABILISE) {
            // ANGLE MODE: Cascaded control with outer angle loop

            // Map remote control inputs to angle setpoints
            double pitchAngleSetpoint =
                mapf(remoteIn.pitch, -MAX_RATE, MAX_RATE, -MAX_ANGLE, MAX_ANGLE);
            double rollAngleSetpoint =
                mapf(remoteIn.roll, -MAX_RATE, MAX_RATE, -MAX_ANGLE, MAX_ANGLE);

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
        outputPID.roll_pid =
            pid_update(&ratePid, &pidRate.roll, rollRateSetpoint, kalman.rollRate);
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
