/*
 ******************************************************************************
 * File: protocol.h
 * Author: Jack Cairns
 * Date: 19-11-2026
 ******************************************************************************
 * Wire-format packet definitions shared by the drone, remote and bridge
 * firmware and by the host-side config GUIs (config-app and config-app-c).
 ******************************************************************************
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

struct sensor_telemetry_t {
    float pitch_angle, roll_angle, yaw_angle;
    float pitch_rate, roll_rate, yaw_rate;
    float mode;
    float pid_pitch, pid_roll, pid_yaw;
    uint16_t motor_a, motor_b, motor_c, motor_d;
    uint16_t battery;
    uint16_t loop_dt_max_us;
    // What the pilot is commanding, as the drone received it (stick -> deg/s, throttle in us)
    float cmd_pitch, cmd_roll, cmd_yaw;
    uint16_t cmd_throttle;
};

struct remote_telemetry_t {
    uint16_t throttle, pitch, roll, yaw;
    uint16_t flight_mode;
};

struct pid_config_telemetry_t {
    float kp, ki, kd;
    float d_alpha; // D-term low-pass coefficient, 0-1 (1 = unfiltered)
    uint16_t axis; // 0 = Pitch & Roll, 1 = Yaw
    uint16_t mode; // 0 = Rate, 1 = Angle
};

struct power_data_telemetry_t {
    uint16_t battery;
};

struct drone_config_telemetry_t {
    float max_rate, max_angle, fail_angle;
    float min_throttle, max_throttle;
    uint32_t coms_timeout_us;
    uint16_t low_voltage, critical_voltage;
};

struct joystick_cal_telemetry_t {
    uint16_t min, centre, max;
};

struct remote_config_telemetry_t {
    float voltage_cal_multiplier;
    uint16_t low_voltage, critical_voltage;
    struct joystick_cal_telemetry_t throttle, pitch, roll, yaw;
};

struct config_ack {
    uint16_t status;
};

union packet_data {
    struct sensor_telemetry_t sensor;
    struct remote_telemetry_t remote;
    struct pid_config_telemetry_t pid_config;
    struct power_data_telemetry_t power;
    struct drone_config_telemetry_t drone_config;
    struct remote_config_telemetry_t remote_config;
    struct config_ack result;
};

struct wifi_packet_t {
    uint8_t packet_id;
    uint16_t crc16;
    union packet_data data;
};

enum esp_now_packet_id {
    SENSOR,
    REMOTE,
    POWER,
    PID_CONFIG,
    PID_ACK,
    DRONE_CFG,
    DRONE_CFG_STORE,
    DRONE_CFG_ACK,
    REMOTE_CFG,
    REMOTE_CFG_STORE,
    REMOTE_CFG_ACK,
};

#endif
