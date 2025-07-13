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

// Common project functions
#include "common.h"

// Custom Components
#include "hmc5883l.h"
#include "l3gd20.h"
#include "lis3DH.h"

#define GYRO_ALPHA 0.30

#define IMU_STACK (configMINIMAL_STACK_SIZE * 2)
#define IMU_PRIO (tskIDLE_PRIORITY + 3)

///////////////////////////// Structures & Enums /////////////////////////////

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
void imu_task(void);
void imu_sign_check_task(void);

extern TaskHandle_t imuTask;
extern QueueHandle_t imuQueue;

#endif
