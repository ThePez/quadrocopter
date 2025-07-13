/*
 *****************************************************************************
 * File: imu.h
 * Author: Jack Cairns
 * Date: 13-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef IMU_H
#define IMU_H

// STD C lib headers
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

// ESP-IDF Prebuilts
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Custom Components
#include "hmc5883l.h"
#include "l3gd20.h"
#include "lis3DH.h"

#define GYRO_ALPHA 0.30

#define IMU_STACK (configMINIMAL_STACK_SIZE * 2)
#define IMU_PRIO (tskIDLE_PRIORITY + 3)

#define IMU_QUEUE_LENGTH 5

///////////////////////////// Structures & Enums /////////////////////////////

typedef struct {
    i2c_master_bus_handle_t* i2cHost;
    SemaphoreHandle_t* spiVMutex;
    SemaphoreHandle_t* spiHMutex;
    SemaphoreHandle_t* i2cMutex;
} ImuParams_t;

typedef struct {
    double pitchAngle;
    double pitchRate;
    double rollAngle;
    double rollRate;
    double yawAngle;
    double yawRate;
    uint64_t prevTime;
} Telemitry_t;

///////////////////////////////// Prototypes /////////////////////////////////

// Task Function Prototypes
void imu_task(void* pvParams);
void imu_sign_check_task(void);

extern TaskHandle_t imuTask;
extern QueueHandle_t imuQueue;

#endif
