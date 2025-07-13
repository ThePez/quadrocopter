/*
 *****************************************************************************
 * File: drone.h
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef DRONE_H
#define DRONE_H

// STD C lib headers
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define SYS_STACK (configMINIMAL_STACK_SIZE * 2)
#define SYS_PRIO (tskIDLE_PRIORITY + 2)

///////////////////////////// Structures & Enums /////////////////////////////

typedef struct {
    double kp;           // Proportional scaler
    double ki;           // Intergral scaler
    double kd;           // Derivative scaler
    double intergral;    // running average total of the error
    double prevError;    // previous error
    uint64_t prevTimeUS; // Last update time in us
} PID_t;

typedef struct {
    double errPitch;
    double errRoll;
    double errYaw;
} ControlError_t;

///////////////////////////////// Prototypes /////////////////////////////////

// Task Function Prototypes
void flight_controller(void);
void update_escs(uint16_t throttle, double pitchPID, double rollPID, double yawPID);
double pid_update(PID_t* pid, double error);

#endif
