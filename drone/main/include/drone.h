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

enum { PID_MODIFIERS, SETPOINT_UPDATE } MessageType_t;

typedef struct {
    double pitch;
    double roll;
    double yaw;
    double throttle;
} RemoteSetPoints_t;

typedef struct {
    double kp;           // Proportional scaler
    double ki;           // Intergral scaler
    double kd;           // Derivative scaler
    double intergral;    // running average total of the error
    double prevError;    // previous error
    uint64_t prevTimeUS; // Last update time in us
} PID_t;

typedef struct {
    PID_t pids[6];
    double errors[6];
} ControlSystem_t;

///////////////////////////////// Prototypes /////////////////////////////////

// Task Function Prototypes
void flight_controller(void);
void process_remote_data(RemoteSetPoints_t* setPoints, ControlSystem_t* system, uint16_t* payload);
double throttle_adc_convert(uint16_t value);
double angle_adc_convert(uint16_t value);
void update_escs(uint16_t throttle, double pitchPID, double rollPID, double yawPID);
double pid_update(PID_t* pid, double error, uint64_t now);
void remote_data_callback(void* args);

#endif
