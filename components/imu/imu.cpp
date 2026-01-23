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

// IMU data containers
static Telemitry_t imuBufA;
static Telemitry_t imuBufB;
volatile Telemitry_t* imuData = &imuBufA;
static Telemitry_t* imuBuffer = &imuBufB;

static void imu_data_callback(BNO08x* imu) {
    if (!imu) {
        return;
    }

    /* No SH2-HAL */
    // get rates
    // imuBuffer->rollRate = imu->get_calibrated_gyro_velocity_X();
    // imuBuffer->pitchRate = imu->get_calibrated_gyro_velocity_Y();
    // imuBuffer->yawRate = imu->get_calibrated_gyro_velocity_Z();

    // // Get angles
    // imuBuffer->rollAngle = imu->get_roll_deg();
    // imuBuffer->pitchAngle = imu->get_pitch_deg();
    // imuBuffer->yawAngle = imu->get_yaw_deg();

    /* With SH2-HAL */
    // Rates
    bno08x_gyro_t velocity = imu->rpt.cal_gyro.get();
    imuBuffer->rollRate = RAD_2_DEG(velocity.x);  // X-axis -> Roll
    imuBuffer->pitchRate = RAD_2_DEG(velocity.y); // Y-axis -> Pitch
    imuBuffer->yawRate = RAD_2_DEG(velocity.z);   // Z-axis -> Yaw

    // Angles
    bno08x_euler_angle_t euler = imu->rpt.rv_ARVR_stabilized_game.get_euler();
    imuBuffer->pitchAngle = euler.x; // X-axis -> Pitch
    imuBuffer->rollAngle = euler.y;  // Y-axis -> Roll
    imuBuffer->yawAngle = euler.z;   // Z-axis -> Yaw

    // Set active buffer
    imuData = imuBuffer;
    // Swap buffer to use on next sensor input
    imuBuffer = (imuBuffer == &imuBufA) ? &imuBufB : &imuBufA;
}

static void bno08x_task(void* pvParameters) {

    BNO08x imu; // create IMU object with default wiring scheme

    if (!imu.initialize()) {
        ESP_LOGE(TAG, "IMU Init Failed");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "IMU Initialised");

    // Enable gyro & ARVR game rotation vector

    /* No SH2-HAL */
    // imu.enable_ARVR_stabilized_game_rotation_vector(10000UL);
    // imu.enable_calibrated_gyro(10000UL);
    
    /* With SH2-HAL */
    imu.rpt.rv_ARVR_stabilized_game.enable(2500UL);
    imu.rpt.cal_gyro.enable(2500UL);
    ESP_LOGI(TAG, "Reports enabled");

    // Register a callback function
    imu.register_cb([&imu]() { imu_data_callback(&imu); });
    ESP_LOGI(TAG, "Callback registered");

    while (1) {
        ESP_LOGI(TAG, "Task suspended");
        vTaskSuspend(NULL); // Susspend task as callback functions handle the data.

        // Emergency shutdown path

        /* No SH2-HAL */
        // imu.disable_ARVR_stabilized_game_rotation_vector();
        // imu.disable_calibrated_gyro();

        /* With SH2-HAL */
        imu.rpt.rv_ARVR_stabilized_game.disable();
        imu.rpt.cal_gyro.disable();

        ESP_LOGW(TAG, "IMU reports disabled, shutting down");
        // Delete the setup task
        vTaskDelete(NULL);
    }
}

void imu_init(void) {
    xTaskCreate(bno08x_task, "IMU Task", BNO085_STACK_SIZE, NULL, BNO085_PRIORITY, &imuTaskHandle);
}

void imu_kill(void) {
    vTaskResume(imuTaskHandle);
}
