/*
 *****************************************************************************
 * File: rx_task.h
 * Author: Jack Cairns
 * Date: 16-07-2026
 *****************************************************************************
 */

#ifndef RX_TASK_H
#define RX_TASK_H

#include <esp_err.h>

/**
 * @brief Creates the RX task.
 *
 * @return ESP_OK if the task was created, ESP_FAIL otherwise.
 */
esp_err_t rx_task_init(void);

#endif
