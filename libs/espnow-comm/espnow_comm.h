/*
 ******************************************************************************
 * File: espnow_comm.h
 * Author: Jack Cairns
 * Date: 17-01-2026
 ******************************************************************************
 */

#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <stdint.h>

struct sensor_telemetry_t {
    float pitch_angle, roll_angle, yaw_angle;
    float pitch_rate, roll_rate, yaw_rate;
    float mode;
    float pid_pitch, pid_roll, pid_yaw;
    uint16_t motor_a, motor_b, motor_c, motor_d;
    uint16_t battery;
};

struct remote_telemetry_t {
    uint16_t throttle, pitch, roll, yaw;
    uint16_t flight_mode;
};

struct pid_config_telemetry_t {
    float kp, ki, kd;
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

union packet_data {
    struct sensor_telemetry_t sensor;
    struct remote_telemetry_t remote;
    struct pid_config_telemetry_t pid_config;
    struct power_data_telemetry_t power;
    struct drone_config_telemetry_t drone_config;
    struct remote_config_telemetry_t remote_config;
};

struct wifi_packet_t {
    union packet_data data;
    uint16_t crc16;
    uint8_t packet_id;
};

enum wifi_packet_id {
    SENSOR,
    REMOTE,
    PID_CONFIG,
    POWER,
    DRONE_CFG,
    DRONE_CFG_STORE,
    REMOTE_CFG,
    REMOTE_CFG_STORE
};

/**
 * @brief Initializes NVS, WiFi, and ESP-NOW, and registers the given peers.
 *
 * This function:
 * - Initializes NVS flash, erasing and reinitializing it if no free pages
 *   are available or a new version is found.
 * - Initializes WiFi in station mode and sets the ESP-NOW channel.
 * - Initializes the ESP-NOW driver, sets the PMK, registers the send/receive
 *   callbacks, and adds each address in peer_addr as an encrypted peer.
 * - Creates the wifiQueue (if not already created) and wifiSendSemaphore
 *   used for queuing and gating outgoing packets.
 *
 * @param peer_addr Array of MAC addresses (6 bytes each) to register as ESP-NOW peers.
 * @param num_peers Number of MAC addresses in peer_addr.
 *
 * @note Must be called once during startup before sending/receiving ESP-NOW packets.
 *
 * @global wifiQueue          Created if it does not already exist.
 * @global wifiSendSemaphore  Created and given so the first send can proceed.
 *
 * @return ESP_OK on success, or error code on failure
 */
esp_err_t esp_now_module_init(const uint8_t* peer_addr[], uint8_t num_peers);

/**
 * @brief Computes and stores the packet's CRC16, then sends it via ESP-NOW.
 *
 * Wraps esp_now_send() so callers don't need to calculate the CRC
 * themselves; the value is written into packet->crc16 over the data
 * union before transmission.
 *
 * @param addr   MAC address (6 bytes) of the peer to send to, or NULL to
 *               send to all registered peers.
 * @param packet Packet to checksum and send. Modified in place (crc16 field).
 *
 * @return ESP_OK on success, or error code from esp_now_send() on failure.
 */
esp_err_t esp_now_send_packet(const uint8_t* addr, struct wifi_packet_t* packet);

// DRONE's MAC ADDRESS
extern const uint8_t drone_mac[6];
// REMOTE's MAC ADDRESS
extern const uint8_t remote_mac[6];
// BRIDGE's MAC ADDRESS
extern const uint8_t bridge_mac[6];

// ESPNOW queue
extern QueueHandle_t wifiQueue;
extern SemaphoreHandle_t wifiSendSemaphore;

#endif