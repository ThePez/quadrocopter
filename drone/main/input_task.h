/*
 *****************************************************************************
 * File: input_task.h
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#ifndef INPUT_TASK_H
#define INPUT_TASK_H

#include <esp_err.h>

/**
 * @brief Starts the task that consumes incoming ESP-NOW packets from wifiQueue.
 *
 * Handles REMOTE packets by mapping raw joystick/throttle ADC values into
 * remoteIn and updating the flight mode; PID_CONFIG packets via
 * pid_handle_config_update(); DRONE_CONFIG packets via
 * drone_config_handle_update(); and CONFIG_SAVE via drone_config_handle_save().
 * Pinned to core 0.
 *
 * @return ESP_OK if the task was created, ESP_FAIL otherwise.
 */
esp_err_t input_task_init(void);

#endif
