/*
 *****************************************************************************
 * File: drone.h
 * Author: Jack Cairns
 * Date: 27-06-2025
 *****************************************************************************
 */

#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "common_functions.h"

#define ADC_MIN 0
#define ADC_MAX 4095

#define MAX_RATE 200.0 // deg/s
#define MAX_ANGLE 25.0 // deg
#define FAIL_ANGLE 30.0

#define MIN_THROTTLE 1000.0 // us
#define MAX_THROTTLE 2000.0 // us

#define FAILSAFE_TIMEOUT_US 2000000 // 1 second

#define ANGLE_FAIL (1 << 0)
#define COMS_FAIL (1 << 1)

// Voltage divider constants
#define R1 46400.0f                            // 46.4k ohms
#define R2 9840.0f                             // 9.84k ohms
#define VOLTAGE_MULTIPLIER ((R1 + R2) / R2)    // ~5.715
#define NON_CALIBRATED_MULTIPLIER 0.000805861f // 3.3 / 4095
#define LOW_VOLTAGE 14000                      // mV (Warn operator at this voltage)
#define CRITICAL_VOLTAGE 13600                 // mV (min voltage the battery can be)

struct target_parameters_t {
    double pitch;
    double roll;
    double yaw;
    double throttle;
};

struct drone_config_t {
    int64_t lastRemoteTime;
    enum flight_mode_t mode;
    uint16_t battery;
    uint8_t armed;
    uint8_t status_mask;
    SemaphoreHandle_t mutex; // Protects the fields above (written from pid_task and input_task on separate cores)
};

// Shared drone/remote-input state (written by input_task, read by pid_task and drone.c)
extern struct drone_config_t droneData;
extern struct target_parameters_t remoteIn;

esp_err_t init_drone(void);

#endif
