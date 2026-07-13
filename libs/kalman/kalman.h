/*
 *****************************************************************************
 * File: kalman.h
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#ifndef KALMAN_H
#define KALMAN_H

#include "imu.h"

#include <esp_err.h>

/**
 * @brief Creates the mutex guarding the shared attitude estimate.
 *
 * Safe to call more than once - subsequent calls are a no-op once the
 * mutex exists. Must be called before kalman_update()/kalman_get() are
 * used from separate tasks.
 *
 * @return ESP_OK on success, ESP_FAIL if mutex creation failed.
 */
esp_err_t kalman_init(void);

/**
 * @brief Filters a raw IMU sample and stores it as the latest attitude estimate.
 *
 * Low-pass filters the pitch/roll/yaw gyro rates in place on the given
 * sample, then also runs a 1D Kalman filter on the pitch/roll angles if
 * enabled (off by default - see the ENABLE define in kalman.c). The result
 * is written into the shared "latest" estimate under kalmanMutex.
 *
 * @param sample IMU sample to filter; its rate (and, if enabled, angle) fields are modified in place.
 */
void kalman_update(struct imu_packet_t* sample);

/**
 * @brief Copies out the most recent filtered attitude estimate.
 *
 * Non-blocking with respect to new sensor data - returns whatever the last
 * kalman_update() produced, so it can safely be called every PID cycle even
 * if no new IMU sample has arrived yet.
 *
 * @param out Destination for the latest imu_packet_t estimate.
 */
void kalman_get(struct imu_packet_t* out);

#endif
