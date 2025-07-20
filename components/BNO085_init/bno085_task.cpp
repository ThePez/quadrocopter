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

static void bno08x_task(void* pvParameters) {

    while (!bno085Queue) {
        bno085Queue = xQueueCreate(BNO085_QUEUE_LENGTH, sizeof(Telemitry_t));
    }

    BNO08x imu; // create IMU object with default wiring scheme

    imu.initialize(); // initialize IMU
    ESP_LOGI(TAG, "Initialised");

    // Enable gyro & game rotation vector
    imu.enable_game_rotation_vector(20000UL); // 20,000us == 20ms report interval
    imu.enable_calibrated_gyro(20000UL);
    ESP_LOGI(TAG, "Reports enabled");
    // register a callback function (in this case a lambda function, but it doesn't have to be)
    imu.register_cb([&imu]() {
        // Callback function contents, executed whenever new data is parsed
        // Absolute heading in degrees and angular velocity in Rad/s
        Telemitry_t data = {
            .pitchAngle = 0,
            .pitchRate = 0,
            .rollAngle = 0,
            .rollRate = 0,
            .yawAngle = 0,
            .yawRate = 0,
            .prevTime = 0,
        };

        data.prevTime = esp_timer_get_time();
        data.rollRate = imu.get_calibrated_gyro_velocity_X();
        data.pitchRate = imu.get_calibrated_gyro_velocity_Y();
        data.yawRate = imu.get_calibrated_gyro_velocity_Z();
        data.rollAngle = imu.get_roll_deg();
        data.pitchAngle = imu.get_pitch_deg();
        data.yawAngle = imu.get_yaw_deg();
        if (bno085Queue) {
            xQueueSendToBack(bno085Queue, &data, pdMS_TO_TICKS(1));
        }
    });

    ESP_LOGI(TAG, "Callback registered");
    while (1) {
        ESP_LOGI(TAG, "Task suspended");
        vTaskSuspend(NULL); // Susspend task as callback functions handle the data.
    }
}

void bno08x_start_task(void) {
    xTaskCreate(bno08x_task, "BNO08x Task", BNO085_STACK_SIZE, NULL, BNO085_PRIORITY, &bno085Task);
}
