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
#include "motors.h"
#include "pid.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>
#include <stdatomic.h>

#define TAG "PID_TASK"

#define PID_TASK_STACK 4096
#define PID_TASK_PRIO  (tskIDLE_PRIORITY + 4)

// Expected time between IMU samples (must match REPORT_FREQUENCY in imu.cpp).
// Only used for the first cycle and to bound the measured dt.
#define NOMINAL_DT_S 0.0025f
#define MIN_DT_S     0.001f
#define MAX_DT_S     0.010f

// If no IMU sample arrives for this long the motors are stopped
#define IMU_TIMEOUT_MS    50
#define IMU_TIMEOUT_TICKS pdMS_TO_TICKS(IMU_TIMEOUT_MS)

// Smallest D-term filter coefficient accepted from the ground station
#define MIN_DTERM_ALPHA 0.01f

// Gyro low-pass coefficient: 0.6 at 400 Hz is a cutoff of roughly 95 Hz
// (alpha = dt / (dt + 1 / (2 * pi * fc))). 1 = unfiltered.
#define GYRO_LPF_ALPHA 0.6f

// Throttle this far above min_throttle is treated as "stick down"
#define IDLE_OFFSET 50.0f

// With a critical battery, in-flight throttle is capped to this fraction of
// the throttle range. Set it just under your hover throttle so the drone sinks.
#define CRITICAL_THROTTLE_FRACTION 0.5f

struct pid_feedback_t {
    struct pid_result_t pitch;
    struct pid_result_t roll;
    struct pid_result_t yaw;
};

// The PID task's private copy of the values it needs from droneCfg, refreshed
// only when the config changes so the loop never touches cfgMutex otherwise.
struct flight_cfg_t {
    float minThrottle;
    float maxThrottle;
    float maxRate;
    float maxAngle;
    float failAngle;
    uint16_t criticalVoltage;
    int64_t comsTimeoutUs;
};

static TaskHandle_t pidTaskHandle = NULL;

// Set by other tasks, consumed by the PID task
static atomic_bool cfgDirty = false;
static atomic_bool gainsDirty = false;

// Longest gap between control cycles since telemetry last asked (us)
static atomic_uint_fast32_t loopDtMaxUs = 0;

// Co-efficients for the PID control loops (owned by the PID task)
static struct pid_parameters_t ratePid;
static struct pid_parameters_t rateZPid;
static struct pid_parameters_t anglePid;

// Running P/I/D state for each of the 5 control loops (owned by the PID task)
static struct pid_feedback_t pidRate;
static struct pid_feedback_t pidAngle;

struct pid_final_t outputPID;
struct pwm_t motors;
struct imu_packet_t attitude;

// Records a control-cycle interval for the loop-timing telemetry.
static void note_loop_dt(uint32_t dtUs)
{
    if (dtUs > atomic_load(&loopDtMaxUs)) {
        atomic_store(&loopDtMaxUs, dtUs);
    }
}

uint16_t pid_task_take_loop_dt_max_us(void)
{
    uint32_t worst = atomic_exchange(&loopDtMaxUs, 0);
    return (worst > UINT16_MAX) ? UINT16_MAX : (uint16_t)worst;
}

// Limits one axis' PID output to the authority it is allowed over the motors.
static float limit_authority(float output)
{
    return constrainf(output, -MAX_MOTOR_ADJUSTMENT, MAX_MOTOR_ADJUSTMENT);
}

// Copies everything the control loop needs out of droneCfg.
static void load_config(struct flight_cfg_t *cfg)
{
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    struct nvs_drone_cfg_t snapshot = droneCfg;
    xSemaphoreGive(cfgMutex);

    cfg->minThrottle = (float)snapshot.min_throttle;
    cfg->maxThrottle = (float)snapshot.max_throttle;
    cfg->maxRate = (float)snapshot.max_rate;
    cfg->maxAngle = (float)snapshot.max_angle;
    cfg->failAngle = (float)snapshot.fail_angle;
    cfg->comsTimeoutUs = snapshot.coms_timeout_us;
    cfg->criticalVoltage = snapshot.critical_voltage;

    pid_init_params(&ratePid, snapshot.rate_pitch_roll.kp, snapshot.rate_pitch_roll.kd,
                    snapshot.rate_pitch_roll.ki, snapshot.rate_pitch_roll.dtermAlpha);
    pid_init_params(&rateZPid, snapshot.rate_yaw.kp, snapshot.rate_yaw.kd, snapshot.rate_yaw.ki,
                    snapshot.rate_yaw.dtermAlpha);
    pid_init_params(&anglePid, snapshot.angle_pitch_roll.kp, snapshot.angle_pitch_roll.kd,
                    snapshot.angle_pitch_roll.ki, snapshot.angle_pitch_roll.dtermAlpha);
}

// Clears the running state of every control loop.
static void reset_controllers(void)
{
    pid_reset(&pidRate.pitch);
    pid_reset(&pidRate.roll);
    pid_reset(&pidRate.yaw);
    pid_reset(&pidAngle.pitch);
    pid_reset(&pidAngle.roll);

    outputPID.pitch_pid = 0.0f;
    outputPID.roll_pid = 0.0f;
    outputPID.yaw_pid = 0.0f;
}

// Drives all 4 ESCs to minimum throttle. Also clears the controllers, so no
// integrator can wind up while the motors are not actually being driven.
static void stop_motors(const struct flight_cfg_t *cfg)
{
    uint16_t minThrottle = (uint16_t)cfg->minThrottle;

    for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++) {
        esc_pwm_set_duty_cycle((MotorIndex)i, minThrottle);
    }

    motors.motorA = minThrottle;
    motors.motorB = minThrottle;
    motors.motorC = minThrottle;
    motors.motorD = minThrottle;

    reset_controllers();
}

// Low-pass filters the gyro rates in place (the BNO085's fused angles are
// used as-is). The first sample seeds the filter.
static void filter_rates(struct imu_packet_t *sample)
{
    static struct imu_packet_t filtered;
    static uint8_t seeded = 0;

    if (!seeded) {
        filtered = *sample;
        seeded = 1;
        return;
    }

    filtered.pitchRate += GYRO_LPF_ALPHA * (sample->pitchRate - filtered.pitchRate);
    filtered.rollRate += GYRO_LPF_ALPHA * (sample->rollRate - filtered.rollRate);
    filtered.yawRate += GYRO_LPF_ALPHA * (sample->yawRate - filtered.yawRate);

    sample->pitchRate = filtered.pitchRate;
    sample->rollRate = filtered.rollRate;
    sample->yawRate = filtered.yawRate;
}

// Latches the angle failsafe (disarms) once pitch/roll exceeds the fail
// angle. No-op if the failsafe is already active.
static void angle_failsafe(void)
{
    xSemaphoreTake(droneData.mutex, portMAX_DELAY);
    if (droneData.status_mask & ANGLE_FAIL) {
        xSemaphoreGive(droneData.mutex);
        return;
    }

    droneData.status_mask |= ANGLE_FAIL;
    droneData.armed = 0;
    xSemaphoreGive(droneData.mutex);

    ESP_LOGE(TAG, "FAILSAFE: angles Pitch %.1f, Roll %.1f", attitude.pitchAngle,
             attitude.rollAngle);
    ESP_LOGW(TAG, "FAILSAFE: Level the drone and throttle down to reset");
}

// Latches the comms failsafe (disarms) once the link to the remote has been
// silent for longer than the configured timeout.
static void comms_failsafe(const struct flight_cfg_t *cfg)
{
    xSemaphoreTake(droneData.mutex, portMAX_DELAY);
    if ((droneData.status_mask & COMS_FAIL) || !droneData.armed) {
        xSemaphoreGive(droneData.mutex);
        return;
    }

    droneData.status_mask |= COMS_FAIL;
    droneData.armed = 0;
    xSemaphoreGive(droneData.mutex);

    ESP_LOGE(TAG, "FAILSAFE: No wifi link for %lld ms", cfg->comsTimeoutUs / 1000);
    ESP_LOGW(TAG, "FAILSAFE: May need to reset both remote and drone to reset");
}

// Decides how much throttle the pilot is allowed, given the battery voltage.
// The returned value replaces the stick throttle for this control cycle;
// anything below (minThrottle + IDLE_OFFSET) stops the motors.
static float battery_throttle_limit(float throttle, uint16_t batteryMv,
                                    const struct flight_cfg_t *cfg)
{
    // Whether the motors were being driven on the previous cycle (PID task only)
    static uint8_t flying = 0;

    float idle = cfg->minThrottle + IDLE_OFFSET;

    if (batteryMv < cfg->criticalVoltage) {
        if (!flying) {
            // On the ground: refuse to take off (also covers the first second
            // after boot, before the first battery reading arrives)
            return cfg->minThrottle;
        }

        // In the air: never cut out. Cap the throttle so the drone can only
        // descend or hold, which also reduces the sag that may have caused this.
        float cap =
            cfg->minThrottle + CRITICAL_THROTTLE_FRACTION * (cfg->maxThrottle - cfg->minThrottle);
        throttle = (throttle > cap) ? cap : throttle;
    }

    flying = throttle >= idle;
    return throttle;
}

// Mixes throttle with the latest PID outputs into each motor's duty cycle
// and writes the results to the ESCs.
static void update_escs(float throttle, const struct flight_cfg_t *cfg)
{
    float idle = cfg->minThrottle + IDLE_OFFSET;
    float max = cfg->maxThrottle;

    float pitch = outputPID.pitch_pid;
    float roll = outputPID.roll_pid;
    float yaw = outputPID.yaw_pid;

    // Front left
    motors.motorA = (uint16_t)constrainf(throttle - pitch + roll - yaw, idle, max);
    // Rear left
    motors.motorB = (uint16_t)constrainf(throttle + pitch + roll + yaw, idle, max);
    // Rear right
    motors.motorC = (uint16_t)constrainf(throttle + pitch - roll - yaw, idle, max);
    // Front right
    motors.motorD = (uint16_t)constrainf(throttle - pitch - roll + yaw, idle, max);

    esc_pwm_set_duty_cycle(MOTOR_A, motors.motorA);
    esc_pwm_set_duty_cycle(MOTOR_B, motors.motorB);
    esc_pwm_set_duty_cycle(MOTOR_C, motors.motorC);
    esc_pwm_set_duty_cycle(MOTOR_D, motors.motorD);
}

void set_flight_mode(enum flight_mode_t mode)
{
    xSemaphoreTake(droneData.mutex, portMAX_DELAY);
    uint8_t changed = (mode != droneData.mode);
    droneData.mode = mode;
    xSemaphoreGive(droneData.mutex);

    // The PID task notices the change itself and resets its controllers
    if (changed) {
        ESP_LOGI(TAG, "Control mode: %s", mode == STABILISE ? "Stabilise" : "Acro");
    }
}

void pid_task_config_changed(void)
{
    atomic_store(&cfgDirty, true);
}

void pid_handle_config_update(struct pid_config_telemetry_t *packet)
{

    const char *axis_names[] = {"Pitch & Roll", "Yaw"};
    const char *mode_names[] = {"Rate", "Angle"};

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

    // Keep the filter coefficient usable (this also catches NaN)
    float dAlpha = packet->d_alpha;
    if (!(dAlpha >= MIN_DTERM_ALPHA)) {
        dAlpha = MIN_DTERM_ALPHA;
    } else if (dAlpha > 1.0f) {
        dAlpha = 1.0f;
    }

    ESP_LOGI(TAG, "PID Update - %s %s: Kp=%.4f Ki=%.4f Kd=%.4f D-alpha=%.3f",
             axis_names[packet->axis], mode_names[packet->mode], packet->kp, packet->ki, packet->kd,
             dAlpha);

    // droneCfg is the single source of truth: write the gains there (so a
    // later CONFIG_SAVE persists them) and let the PID task pick them up.
    struct nvs_pid_cfg_t *gains;
    if (packet->mode) {
        gains = &droneCfg.angle_pitch_roll;
    } else {
        gains = (packet->axis == 1) ? &droneCfg.rate_yaw : &droneCfg.rate_pitch_roll;
    }

    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    gains->kp = packet->kp;
    gains->ki = packet->ki;
    gains->kd = packet->kd;
    gains->dtermAlpha = dAlpha;
    xSemaphoreGive(cfgMutex);

    // New gains also clear the integrators
    atomic_store(&gainsDirty, true);
    atomic_store(&cfgDirty, true);
}

// Main PID task loop: runs once per IMU sample. Checks failsafes, runs the
// cascaded rate/angle PID controllers for the current flight mode, and pushes
// the results to the ESCs.
static void pid_control(void *pvParams)
{
    ARG_UNUSED(pvParams);

    // The IMU task creates its queue asynchronously
    while (!imuQueue) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    struct flight_cfg_t cfg;
    load_config(&cfg);
    reset_controllers();

    ESP_LOGI(TAG, "PID Task Setup");

    struct imu_packet_t sample;
    enum flight_mode_t lastMode = ACRO;
    int64_t lastSampleUs = 0;
    uint8_t imuStalled = 0;

    while (1) {
        // Each new IMU sample paces one control cycle
        if (xQueueReceive(imuQueue, &sample, IMU_TIMEOUT_TICKS) != pdTRUE) {
            if (!imuStalled) {
                ESP_LOGE(TAG, "FAILSAFE: IMU stopped reporting");
                imuStalled = 1;
            }

            stop_motors(&cfg);
            note_loop_dt(IMU_TIMEOUT_MS * 1000);
            lastSampleUs = 0;
            continue;
        }

        imuStalled = 0;

        int64_t now = esp_timer_get_time();
        float dt = NOMINAL_DT_S;
        if (lastSampleUs != 0) {
            note_loop_dt((uint32_t)(now - lastSampleUs));
            dt = constrainf((float)(now - lastSampleUs) * 1e-6f, MIN_DT_S, MAX_DT_S);
        }

        lastSampleUs = now;

        // Pick up any config/gain changes made by the input task
        if (atomic_exchange(&cfgDirty, false)) {
            load_config(&cfg);
        }

        if (atomic_exchange(&gainsDirty, false)) {
            reset_controllers();
        }

        filter_rates(&sample);
        attitude = sample;

        // Check drone for critical angles
        if (fabsf(attitude.pitchAngle) > cfg.failAngle ||
            fabsf(attitude.rollAngle) > cfg.failAngle) {
            angle_failsafe();
            stop_motors(&cfg);
            continue;
        }

        // Snapshot the shared fields once per iteration under the lock, then make
        // decisions off the local copies so the mutex isn't held for the whole loop body
        xSemaphoreTake(droneData.mutex, portMAX_DELAY);
        uint8_t statusMask = droneData.status_mask;
        int64_t lastRemoteTime = droneData.lastRemoteTime;
        uint8_t armed = droneData.armed;
        uint16_t battery = droneData.battery;
        enum flight_mode_t mode = droneData.mode;
        xSemaphoreGive(droneData.mutex);

        float idle = cfg.minThrottle + IDLE_OFFSET;

        if (statusMask & ANGLE_FAIL) {
            // To reset drone, throttle must be off and drone must be level
            uint8_t throttle_safe = remoteIn.throttle < idle;
            uint8_t level = (fabsf(attitude.pitchAngle) < 5) && (fabsf(attitude.rollAngle) < 5);
            if (throttle_safe && level) {
                ESP_LOGI(TAG, "Angle Failsafe cleared");
                xSemaphoreTake(droneData.mutex, portMAX_DELAY);
                droneData.status_mask &= ~ANGLE_FAIL;
                xSemaphoreGive(droneData.mutex);
            } else {
                stop_motors(&cfg);
                continue;
            }
        }

        // Check last connection to remote
        if (now - lastRemoteTime > cfg.comsTimeoutUs) {
            comms_failsafe(&cfg);
            stop_motors(&cfg);
            continue;
        } else if (statusMask & COMS_FAIL) {
            // Comms recovered - clear failsafe
            xSemaphoreTake(droneData.mutex, portMAX_DELAY);
            droneData.status_mask &= ~COMS_FAIL;
            xSemaphoreGive(droneData.mutex);
            ESP_LOGI(TAG, "Comms failsafe cleared - link restored");
        }

        // Only run control if armed, no failsafes active, and the stick is up.
        // Sitting on the ground must not accumulate any controller state.
        float throttle = battery_throttle_limit(remoteIn.throttle, battery, &cfg);
        if (!armed || throttle < idle) {
            stop_motors(&cfg);
            continue;
        }

        // Start from a clean slate when the flight mode changes
        if (mode != lastMode) {
            reset_controllers();
            lastMode = mode;
        }

        float pitchRateSetpoint, rollRateSetpoint;

        if (mode == STABILISE) {
            // ANGLE MODE: Cascaded control with outer angle loop

            // Map remote control inputs to angle setpoints
            float pitchAngleSetpoint =
                mapf(remoteIn.pitch, -cfg.maxRate, cfg.maxRate, -cfg.maxAngle, cfg.maxAngle);
            float rollAngleSetpoint =
                mapf(remoteIn.roll, -cfg.maxRate, cfg.maxRate, -cfg.maxAngle, cfg.maxAngle);

            pitchRateSetpoint =
                pid_update(&anglePid, &pidAngle.pitch, pitchAngleSetpoint, attitude.pitchAngle, dt);
            rollRateSetpoint =
                pid_update(&anglePid, &pidAngle.roll, rollAngleSetpoint, attitude.rollAngle, dt);

        } else {
            // RATE MODE: Direct rate control
            pitchRateSetpoint = remoteIn.pitch;
            rollRateSetpoint = remoteIn.roll;
        }

        // Yaw is still direct rate control in all control modes
        float yawRateSetpoint = remoteIn.yaw;

        // Each axis may only move a motor by MAX_MOTOR_ADJUSTMENT, so a bad
        // gain can't swing the motors across their whole range
        outputPID.pitch_pid = limit_authority(
            pid_update(&ratePid, &pidRate.pitch, pitchRateSetpoint, attitude.pitchRate, dt));
        outputPID.roll_pid = limit_authority(
            pid_update(&ratePid, &pidRate.roll, rollRateSetpoint, attitude.rollRate, dt));
        outputPID.yaw_pid = limit_authority(
            pid_update(&rateZPid, &pidRate.yaw, yawRateSetpoint, attitude.yawRate, dt));

        // Update ESCs
        update_escs(throttle, &cfg);
    }
}

esp_err_t pid_task_init(void)
{
    BaseType_t result = xTaskCreatePinnedToCore(pid_control, "PID_TASK", PID_TASK_STACK, NULL,
                                                PID_TASK_PRIO, &pidTaskHandle, (BaseType_t)1);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
