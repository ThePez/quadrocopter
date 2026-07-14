/*
 *****************************************************************************
 * File: remote.h
 * Author: Jack Cairns
 * Date: 23-07-2025
 *****************************************************************************
 */

#ifndef REMOTE_H
#define REMOTE_H

#include "device_config.h"

#include <esp_err.h>

#define MODE_BUTTON_PIN 25
#define SHUTOFF_BUTTON_PIN 32

extern struct nvs_remote_cfg_t remoteCfg;

/**
 * @brief Starts the task that runs the whole remote (buttons, joystick ADC, ESP-NOW link).
 *
 * The spawned task performs hardware init (ESP-NOW paired to the drone,
 * mode/emergency button interrupts, the MCP3208 ADC task, and the polling
 * timer), then loops handling button presses and periodic joystick reads,
 * sending a REMOTE packet to the drone once armed.
 *
 * @return ESP_OK if the task was created, ESP_FAIL otherwise.
 */
esp_err_t init_remote(void);

#endif
