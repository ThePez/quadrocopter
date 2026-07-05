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

esp_err_t pid_task_init(void);
void set_flight_mode(enum flight_mode_t mode);
void pid_handle_config_update(struct pid_config_telemetry_t* packet);

#endif
