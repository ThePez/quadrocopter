/*
 *****************************************************************************
 * File: imu.cpp
 * Author: Jack Cairns
 * Date: 20-11-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "imu.h"
#include "BNO08x.hpp"
#include "esp_timer.h"

#define RAD_2_DEG(x) ((180.0 / M_PI) * x)
#define DEG_2_RAD(x) ((M_PI / 180.0) * x)

static const char* TAG = "IMU";
TaskHandle_t imuTaskHandle = NULL;
QueueHandle_t imuQueue = NULL;

/**
 * @brief Callback function for handling BNO08x IMU data.
 *
 * This function is executed when new sensor data is available from the IMU.
 * It collects pitch, roll, and yaw angle and rate, timestamps the sample,
 * and sends the data to the global telemetry queue.
 *
 * @param imu Pointer to the BNO08x instance providing the data.
 */
static void imu_data_callback(BNO08x* imu) {
    if (!imu)
        return;

    Telemitry_t data;

    // Gyro rates
    bno08x_gyro_t velocity = imu->rpt.cal_gyro.get();
    data.rollRate = RAD_2_DEG(velocity.x); // X-axis -> Roll
    data.pitchRate = RAD_2_DEG(velocity.y); // Y-axis -> Pitch
    data.yawRate = RAD_2_DEG(velocity.z); // Z-axis -> Yaw

    bno08x_euler_angle_t euler = imu->rpt.rv_ARVR_stabilized_game.get_euler();
    data.rollAngle = euler.x; // X-axis -> Roll
    data.pitchAngle = euler.y; // Y-axis -> Pitch
    data.yawAngle = euler.z;   // Z-axis -> Yaw

    if (imuQueue) {
        xQueueSendToBack(imuQueue, &data, 0);
    }
}

/**
 * @brief Task function for managing BNO08x IMU data acquisition.
 *
 * Initializes the BNO08x IMU and sets up a callback function that is
 * triggered whenever new sensor data is available. The callback extracts
 * pitch, roll, and yaw angles and angular velocities, then sends this
 * data to a FreeRTOS queue for use elsewhere in the system.
 *
 * The task suspends itself after setup, as the registered callback handles
 * all subsequent data processing.
 *
 * @param pvParameters Unused parameter.
 *
 * @note Relies on a lambda callback registered with the IMU driver.
 */
static void bno08x_task(void* pvParameters) {

    while (!imuQueue) {
        imuQueue = xQueueCreate(BNO085_QUEUE_LENGTH, sizeof(Telemitry_t));
    }

    BNO08x imu; // create IMU object with default wiring scheme

    if (!imu.initialize()) {
        ESP_LOGE(TAG, "IMU Init Failed");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "IMU Initialised");

    // Enable gyro & ARVR game rotation vector
    imu.rpt.rv_ARVR_stabilized_game.enable(10000UL);
    imu.rpt.cal_gyro.enable(10000UL);
    ESP_LOGI(TAG, "Reports enabled");
    
    // Register a callback function
    imu.register_cb([&imu]() { imu_data_callback(&imu); });
    ESP_LOGI(TAG, "Callback registered");
    
    while (1) {
        ESP_LOGI(TAG, "Task suspended");
        vTaskSuspend(NULL); // Susspend task as callback functions handle the data.

        // Emergency shutdown path
        imu.rpt.rv_ARVR_stabilized_game.disable();
        imu.rpt.cal_gyro.disable();

        ESP_LOGW(TAG, "IMU reports disabled, shutting down");
        // Delete the setup task
        vTaskDelete(NULL);
    }
}

/**
 * @brief Starts the BNO08x IMU task.
 *
 * Creates a FreeRTOS task that initializes the IMU, registers a data
 * callback, and sets up data transmission via a queue.
 *
 * @note Uses global task handle `imuTaskHandle`.
 */
void imu_init(void) {
    xTaskCreate(bno08x_task, "IMU Task", BNO085_STACK_SIZE, NULL, BNO085_PRIORITY, &imuTaskHandle);
}

/**
 * @brief Resumes the setup task for the bno085 so that it can shutdown
 */
void imu_kill(void) {
    vTaskResume(imuTaskHandle);
}
