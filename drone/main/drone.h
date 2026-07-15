/*
 *****************************************************************************
 * File: drone.h
 * Author: Jack Cairns
 * Date: 27-06-2025
 *****************************************************************************
 */

#ifndef DRONE_H
#define DRONE_H

#include "common_functions.h"
#include "device_config.h"
#include "espnow_comm.h"

#include <stdint.h>

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define ADC_MIN 0
#define ADC_MAX 4095

#define ANGLE_FAIL (1 << 0)
#define COMS_FAIL (1 << 1)

// Voltage divider constants
#define R1 46400.0f                            // 46.4k ohms
#define R2 9840.0f                             // 9.84k ohms
#define VOLTAGE_MULTIPLIER ((R1 + R2) / R2)    // ~5.715
#define NON_CALIBRATED_MULTIPLIER 0.000805861f // 3.3 / 4095

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
    SemaphoreHandle_t mutex;
};

// Shared drone/remote-input state (written by input_task, read by pid_task and drone.c)
extern struct drone_config_t droneData;
extern struct target_parameters_t remoteIn;
extern struct nvs_drone_cfg_t droneCfg;

/**
 * @brief Brings up the full drone flight system.
 *
 * Initializes ESC PWM output, the battery-voltage ADC, the IMU task, ESP-NOW
 * (paired with the remote and bridge), the droneData/kalman mutexes, the
 * periodic telemetry and battery-check timer callbacks, and finally starts
 * the input and PID control tasks.
 *
 * @return ESP_OK on success, or an error code from the first failing step.
 */
esp_err_t init_drone(void);

/**
 * @brief Applies a DRONE_CONFIG update (failsafe/throttle/battery constants) live.
 *
 * Writes straight into droneCfg under cfgMutex; does not touch flash. A
 * later drone_config_handle_save() is what persists the change.
 *
 * @param packet New values to apply.
 */
esp_err_t drone_config_handle_update(struct drone_config_telemetry_t* packet);

/**
 * @brief Persists the currently-live droneCfg (gains included) to flash.
 *
 * Snapshots droneCfg under cfgMutex, then hands that snapshot to
 * device_config_save() as the new flash contents.
 */
esp_err_t drone_config_handle_save(void);

/**
 * @brief Brings up ESC programming mode instead of normal flight.
 *
 * Drives all 4 ESCs to max throttle immediately on startup (required by most
 * ESCs to enter their programming menu), pairs ESP-NOW with the remote only,
 * and starts the input task plus a task that mirrors the remote's throttle
 * input straight to the ESCs, bypassing flight control entirely.
 *
 * @return ESP_OK on success, or an error code from the first failing step.
 */
esp_err_t init_esc_programming(void);

#endif
