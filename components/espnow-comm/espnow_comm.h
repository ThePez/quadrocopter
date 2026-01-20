/*
 ******************************************************************************
 * File: espnow_comm.h
 * Author: Jack Cairns
 * Date: 17-01-2026
 * Brief:
 * REFERENCE: None
 ******************************************************************************
 */

#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t mac[6];
    int frequency;
} Task_params_t;

typedef struct {
    int16_t pitch_angle;
    int16_t roll_angle;
    int16_t yaw_angle;
    int16_t pitch_rate;
    int16_t roll_rate;
    int16_t yaw_rate;
    int16_t mode;
    int16_t pid_pitch;
    int16_t pid_roll;
    int16_t pid_yaw;
    uint16_t motor_a;
    uint16_t motor_b;
    uint16_t motor_c;
    uint16_t motor_d;
    uint16_t reserved[2];
} __attribute__((packed)) drone_telemetry_packet_t;

typedef struct {
    uint16_t command_id;
    uint16_t throttle;
    uint16_t pitch;
    uint16_t roll;
    uint16_t yaw;
    uint16_t flight_mode;
    uint16_t reserved[10];
} __attribute__((packed)) remote_control_packet_t;

esp_err_t esp_now_module_init(const uint8_t peer_addr[6]);
esp_err_t esp_send_packet(void* packet, const uint8_t len);

// DRONE's MAC ADDRESS
extern const uint8_t drone_mac[6];
// REMOTE's MAC ADDRESS
extern const uint8_t remote_mac[6];
// BRIDGE's MAC ADDRESS
extern const uint8_t bridge_mac[6];

// Encription keys
extern const uint8_t pmk_key[16];
extern const uint8_t lmk_key[16];

// ESPNOW queue
extern QueueHandle_t wifiQueue;

#endif