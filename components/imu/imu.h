/*
 *****************************************************************************
 * File: imu.h
 * Author: Jack Cairns
 * Date: 20-07-2025
 *****************************************************************************
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_err.h>
#include <freertos/FreeRTOS.h>

struct imu_packet_t {
    float pitchAngle; // deg
    float pitchRate;  // deg/s
    float rollAngle;  // deg
    float rollRate;   // deg/s
    float yawAngle;   // deg
    float yawRate;    // deg/s
};

/**
 * @brief Starts the FreeRTOS task that initializes and services the BNO08x IMU.
 *
 * Spawns a task which creates imuQueue, performs the IMU hardware init,
 * enables the gyro-integrated rotation vector report, and registers a
 * callback that publishes an imu_packet_t to imuQueue on every report.
 * imuQueue only ever holds the newest sample (older ones are overwritten).
 *
 * @note imuQueue is not valid until the spawned task has run - callers
 *       should wait briefly before using it.
 *
 * @return ESP_OK if the task was created, ESP_FAIL otherwise.
 */
esp_err_t imu_init(void);

extern QueueHandle_t imuQueue;

#ifdef __cplusplus
}
#endif
#endif
