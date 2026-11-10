/*
 *****************************************************************************
 * File: pid_task.h
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#ifndef PID_TASK_H
#define PID_TASK_H

#include "common_functions.h"
#include "espnow_comm.h"
#include "imu.h"

#include <esp_err.h>

struct pid_final_t {
    float pitch_pid;
    float roll_pid;
    float yaw_pid;
};

struct pwm_t {
    uint16_t motorA;
    uint16_t motorB;
    uint16_t motorC;
    uint16_t motorD;
};

// Latest attitude / PID output / ESC duty cycles. Written only by the PID task;
// other tasks may read them for telemetry (each field is a single atomic word).
extern struct pid_final_t outputPID;
extern struct pwm_t motors;
extern struct imu_packet_t attitude;

/**
 * @brief Starts the PID control task.
 *
 * Pins the control task to core 1. The task runs one control cycle per IMU
 * sample (it blocks on imuQueue) - each cycle filters the gyro rates, checks
 * failsafes, runs the rate/angle PID loops, and updates the ESC duty cycles.
 * If the IMU stops reporting, the motors are stopped.
 *
 * @return ESP_OK if the task was created, ESP_FAIL otherwise.
 */
esp_err_t pid_task_init(void);

/**
 * @brief Switches the active flight mode (ACRO vs STABILISE).
 *
 * No-op if mode already matches droneData.mode. On an actual change the PID
 * task resets all rate and angle controllers on its next cycle, to avoid
 * windup carrying over between modes.
 *
 * @param mode New flight mode to apply.
 */
void set_flight_mode(enum flight_mode_t mode);

/**
 * @brief Applies a PID gain update received from the ground-station over ESP-NOW/UART.
 *
 * Validates packet->axis (0 = Pitch & Roll, 1 = Yaw) and packet->mode
 * (0 = Rate, 1 = Angle), forcing Yaw back to Rate mode since Angle mode is
 * only supported for Pitch/Roll. Writes the new kp/ki/kd and D-term filter
 * coefficient (clamped to a usable range) into droneCfg; the
 * PID task then reloads its gains and resets its controllers on its next cycle.
 *
 * @param packet Gain update to apply; packet->mode may be corrected in place.
 */
void pid_handle_config_update(struct pid_config_telemetry_t *packet);

/**
 * @brief Tells the PID task that droneCfg has changed.
 *
 * The PID task works from its own copy of the config; call this after
 * writing droneCfg so it re-reads the values on its next cycle.
 */
void pid_task_config_changed(void);

/**
 * @brief Returns the longest gap between control cycles since the last call.
 *
 * Reading it resets the measurement. A healthy loop reports roughly the IMU
 * report period (2500 us); an IMU stall reports the stall timeout.
 *
 * @return Worst-case interval in microseconds, saturated at 65535.
 */
uint16_t pid_task_take_loop_dt_max_us(void);

#endif
