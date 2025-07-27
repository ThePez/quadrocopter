/*
 *****************************************************************************
 * File: bno085_task.c
 * Author: Jack Cairns
 * Date: 20-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "bno085_task.h"
#include "BNO08x.hpp"
#include "esp_timer.h"

static const char* TAG = "BNO08xTask";
TaskHandle_t bno085Task = NULL;
QueueHandle_t bno085Queue = NULL;

/**
 * @brief Callback function for handling BNO08x IMU data.
 *
 * This function is executed when new sensor data is available from the IMU.
 * It collects pitch, roll, and yaw angle and rate, timestamps the sample,
 * and sends the data to the global telemetry queue.
 *
 * @param imu Pointer to the BNO08x instance providing the data.
 */
static void bno08x_data_callback(BNO08x* imu) {
    if (!imu)
        return;

    Telemitry_t data;
    data.prevTime = esp_timer_get_time();
    data.rollRate = imu->get_calibrated_gyro_velocity_X();
    data.pitchRate = imu->get_calibrated_gyro_velocity_Y();
    data.yawRate = imu->get_calibrated_gyro_velocity_Z();
    data.rollAngle = imu->get_roll_deg();
    data.pitchAngle = imu->get_pitch_deg();
    data.yawAngle = imu->get_yaw_deg();

    if (bno085Queue) {
        xQueueSendToBack(bno085Queue, &data, pdMS_TO_TICKS(1));
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

    while (!bno085Queue) {
        bno085Queue = xQueueCreate(BNO085_QUEUE_LENGTH, sizeof(Telemitry_t));
    }

    BNO08x imu; // create IMU object with default wiring scheme
    
    imu.initialize(); // initialize IMU
    ESP_LOGI(TAG, "IC Initialised");

    // Enable gyro & game rotation vector
    imu.enable_game_rotation_vector(20000UL); // 20,000us == 20ms report interval
    imu.enable_calibrated_gyro(20000UL);
    ESP_LOGI(TAG, "Reports enabled");
    // Register a callback function
    imu.register_cb([&imu]() { bno08x_data_callback(&imu); });

    ESP_LOGI(TAG, "Callback registered");
    while (1) {
        ESP_LOGI(TAG, "Task suspended");
        vTaskSuspend(NULL); // Susspend task as callback functions handle the data.
    }
}

/**
 * @brief Starts the BNO08x IMU task.
 *
 * Creates a FreeRTOS task that initializes the IMU, registers a data
 * callback, and sets up data transmission via a queue.
 *
 * @note Uses global task handle `bno085Task`.
 */
void bno08x_start_task(void) {
    xTaskCreate(bno08x_task, "BNO08x Task", BNO085_STACK_SIZE, NULL, BNO085_PRIORITY, &bno085Task);
}

void bno085_kill(void) {
}
