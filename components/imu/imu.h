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
    double pitchAngle;
    double pitchRate;
    double rollAngle;
    double rollRate;
    double yawAngle;
    double yawRate;
};

esp_err_t imu_init(void);

extern QueueHandle_t imuQueue;

#ifdef __cplusplus
}
#endif
#endif
