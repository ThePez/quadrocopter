/*
 ******************************************************************************
 * File: device_config.h
 * Author: Jack Cairns
 * Date: 15-07-2026
 ******************************************************************************
 */

#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include <esp_err.h>
#include <stdint.h>

/**
 * @brief Per-device configuration blobs persisted in NVS.
 *
 * nvs_drone_cfg_t and nvs_remote_cfg_t are each written and read as a single
 * blob under one namespace/key (see device_config_load) rather than as many
 * individual keys, so a save is one commit instead of several. Bump
 * `version` on any layout change - device_config_load's version check will
 * then fail against a blob written by the old layout, letting it reseed
 * from compiled defaults instead of reading a stale struct.
 */

// One PID controller's gains (one axis/mode pair).
struct __packed nvs_pid_cfg_t {
    double kp, ki, kd;
    double dtermAlpha;
};

// Calibrated raw ADC range for one joystick channel, recorded by a
// calibration routine rather than assumed, to correct for per-unit
// centering/tolerance drift before the reading is used.
struct __packed nvs_joystick_cal_t {
    uint16_t min, centre, max;
};

struct __packed nvs_drone_cfg_t {
    uint16_t version;
    struct nvs_pid_cfg_t rate_pitch_roll, rate_yaw, angle_pitch_roll;
    double max_rate, max_angle, fail_angle;
    double min_throttle, max_throttle;
    int64_t coms_timeout_us;
    uint16_t low_voltage, critical_voltage;
};

struct __packed nvs_remote_cfg_t {
    uint16_t version;
    double voltage_cal_multiplier;
    uint16_t low_voltage, critical_voltage;
    struct nvs_joystick_cal_t throttle, pitch, roll, yaw;
};

#define NVS_DRONE_CFG_VERSION 1
#define NVS_REMOTE_CFG_VERSION 1

/**
 * @brief Initializes the NVS flash partition.
 *
 * Erases and reinitializes the partition if no free pages are available or
 * a newer NVS version is found. Safe to call more than once - subsequent
 * calls are a no-op once initialization has succeeded.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t nvs_init(void);

/**
 * @brief Loads a versioned config blob from NVS into `cfg`, or seeds it
 * from `defaults` (and writes that back) if nothing valid is there yet.
 *
 * Falls back to defaults if the read fails for any reason (not found, size
 * mismatch from a layout change) or if the stored blob's version field
 * doesn't match `version`.
 *
 * @param namespace NVS namespace to open.
 * @param cfg       Destination buffer; must start with a uint16_t version
 *                  field and be at least `len` bytes.
 * @param len       Size of the config blob in bytes.
 * @param version   Expected value of cfg's leading version field.
 * @param defaults  Compiled-default blob to fall back to, `len` bytes.
 *
 * @return ESP_OK once `cfg` holds a valid config, either way.
 */
esp_err_t device_config_load(const char* namespace, void* cfg, size_t len, uint16_t version,
                             const void* defaults);

#endif
