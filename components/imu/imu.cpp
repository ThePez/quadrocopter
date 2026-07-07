/*
 *****************************************************************************
 * File: imu.cpp
 * Author: Jack Cairns
 * Date: 20-11-2025
 *****************************************************************************
 */

#include "imu.h"

#include "BNO08x.hpp"

#include <freertos/queue.h>
#include <math.h>

#define ENABLE_SH2_HAL

#define REPORT_FREQUENCY 2500UL // 2.5ms between readings

#define IMU_QUEUE_LEN 10
#define IMU_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define IMU_PRIORITY (tskIDLE_PRIORITY + 4)

#define RAD_2_DEG(x) ((180.0 / M_PI) * x)

#define TAG "IMU"

QueueHandle_t imuQueue = NULL;

// Create IMU object with default wiring scheme
static BNO08x imu;

static void imu_data_callback(void) {

    struct imu_packet_t packet;

    // X-axis -> Roll
    // Y-axis -> Pitch
    // Z-axis -> Yaw
#ifdef ENABLE_SH2_HAL
    bno08x_ang_vel_t velocity = imu.rpt.rv_gyro_integrated.get_vel();
    packet.rollRate = RAD_2_DEG(velocity.x);
    packet.pitchRate = RAD_2_DEG(velocity.y);
    packet.yawRate = RAD_2_DEG(velocity.z);

    bno08x_euler_angle_t euler = imu.rpt.rv_gyro_integrated.get_euler(true);
    packet.pitchAngle = euler.x; // X-axis -> Pitch
    packet.rollAngle = euler.y;  // Y-axis -> Roll
    packet.yawAngle = euler.z;   // Z-axis -> Yaw

#else
    packet.rollRate = imu.get_calibrated_gyro_velocity_X();
    packet.pitchRate = imu.get_calibrated_gyro_velocity_Y();
    packet.yawRate = imu.get_calibrated_gyro_velocity_Z();

    packet.rollAngle = imu.get_roll_deg();
    packet.pitchAngle = imu.get_pitch_deg();
    packet.yawAngle = imu.get_yaw_deg();
#endif

    // Post to queue
    xQueueSendToBack(imuQueue, &packet, 0);
}

static void bno08x_task(void* pvParameters) {

    imuQueue = xQueueCreate(IMU_QUEUE_LEN, sizeof(struct imu_packet_t));
    if (!imuQueue) {
        ESP_LOGE(TAG, "Imu Queue failed init");
        vTaskDelete(NULL);
    }

    if (!imu.initialize()) {
        ESP_LOGE(TAG, "Imu failed hardware init");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Imu Device Initialised");
    // Enable the gyro-integrated rotation vector (angle + rate in one report)

#ifdef ENABLE_SH2_HAL
    imu.rpt.rv_gyro_integrated.enable(REPORT_FREQUENCY);
#else
    imu.enable_rv_gyro_integrated(REPORT_FREQUENCY);
#endif
    ESP_LOGI(TAG, "Reports enabled");

    // Register a callback function
    imu.register_cb([]() { imu_data_callback(); });
    ESP_LOGI(TAG, "Callback registered");

    ESP_LOGI(TAG, "Imu setup complete - Task deleted");
    vTaskDelete(NULL);
}

esp_err_t imu_init(void) {
    BaseType_t err = xTaskCreate(bno08x_task, "IMU Task", IMU_STACK_SIZE, NULL, IMU_PRIORITY, NULL);
    return (err == pdPASS) ? ESP_OK : ESP_FAIL;
}
