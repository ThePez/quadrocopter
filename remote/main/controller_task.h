/*
 *****************************************************************************
 * File: controller_task.h
 * Author: Jack Cairns
 * Date: 16-07-2026
 *****************************************************************************
 */

#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define MODE_BUTTON_BIT (1UL << 0)
#define EMERGENCY_BUTTON_BIT (1UL << 1)
#define TIMER_BIT (1UL << 31)

extern TaskHandle_t remoteTaskHandle;

/**
 * @brief Creates the controller task.
 *
 * @return ESP_OK if the task was created, ESP_FAIL otherwise.
 */
esp_err_t controller_task_init(void);

#endif
