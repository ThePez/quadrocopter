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

#include <esp_err.h>

struct pid_final_t {
    double pitch_pid;
    double roll_pid;
    double yaw_pid;
};

struct pwm_t {
    uint16_t motorA;
    uint16_t motorB;
    uint16_t motorC;
    uint16_t motorD;
};

// Latest PID output / ESC duty cycles, reported out by the telemetry callback
extern struct pid_final_t outputPID;
extern struct pwm_t motors;

/**
 * @brief Starts the PID control task and the timer that drives its loop rate.
 *
 * Pins the control task to core 1 and starts an ISR timer (PID_LOOP_FREQ)
 * that notifies it once per control cycle - each cycle reads the latest
 * kalman estimate, checks failsafes, runs the rate/angle PID loops, and
 * updates the ESC duty cycles.
 *
 * @return ESP_OK if both the task and timer were created, ESP_FAIL otherwise.
 */
esp_err_t pid_task_init(void);

/**
 * @brief Switches the active flight mode (ACRO vs STABILISE).
 *
 * No-op if mode already matches droneData.mode. On an actual change, resets
 * all rate and angle PID integrators to avoid windup carrying over between
 * modes.
 *
 * @param mode New flight mode to apply.
 */
void set_flight_mode(enum flight_mode_t mode);

/**
 * @brief Applies a PID gain update received from the ground-station over ESP-NOW/UART.
 *
 * Validates packet->axis (0 = Pitch & Roll, 1 = Yaw) and packet->mode
 * (0 = Rate, 1 = Angle), forcing Yaw back to Rate mode since Angle mode is
 * only supported for Pitch/Roll. Writes the new kp/ki/kd into the matching
 * PID controller and resets the affected integrator(s).
 *
 * @param packet Gain update to apply; packet->mode may be corrected in place.
 */
void pid_handle_config_update(struct pid_config_telemetry_t* packet);

#endif
