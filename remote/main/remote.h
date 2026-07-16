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
#include "espnow_comm.h"

#include <esp_err.h>

#define MODE_BUTTON_PIN 25
#define SHUTOFF_BUTTON_PIN 32

extern struct nvs_remote_cfg_t remoteCfg;

/**
 * @brief Applies a REMOTE_CFG update (joystick calibration/battery constants) live.
 *
 * Writes straight into remoteCfg under cfgMutex; does not touch flash. A
 * later remote_config_handle_save() is what persists the change.
 *
 * @param packet New values to apply.
 */
esp_err_t remote_config_handle_update(struct remote_config_telemetry_t* packet);

/**
 * @brief Persists the currently-live remoteCfg to flash.
 *
 * Snapshots remoteCfg under cfgMutex, then hands that snapshot to
 * device_config_save() as the new flash contents.
 */
esp_err_t remote_config_handle_save(void);

/**
 * @brief Performs hardware init and starts the tasks that run the remote.
 *
 * Synchronously initialises ESP-NOW (paired to the drone and bridge), loads
 * remoteCfg from flash (falling back to defaults on first boot or a struct
 * version mismatch), sets up the mode/emergency button interrupts, the SPI
 * bus and MCP3208 ADC task, and the polling timer. It then spawns two tasks:
 * cfg_task, which applies/persists REMOTE_CFG and REMOTE_CFG_STORE packets
 * received over ESP-NOW, and remote_controller, which loops handling button
 * presses and periodic joystick reads, sending a REMOTE packet to the drone
 * once armed.
 *
 * @return ESP_OK if hardware init succeeded and both tasks were created,
 *         an esp_err_t error code otherwise.
 */
esp_err_t init_remote(void);

#endif
