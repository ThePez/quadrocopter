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
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

// KConfig header
#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define SYS_STACK (configMINIMAL_STACK_SIZE * 2)
#define SYS_PRIO (tskIDLE_PRIORITY + 2)

#define RAD_2_DEG (180.0 / M_PI)
#define DEG_2_RAD (M_PI / 180.0)

#define ADC_MIN 0
#define ADC_MAX 4095

#define MIN_RATE -200.0
#define MAX_RATE 200.0

#define MAX_ANGLE 30.0
#define MIN_ANGLE -30.0

#define MIN_THROTTLE 1000.0
#define MAX_THROTTLE 2000.0

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

///////////////////////////////// Prototypes /////////////////////////////////

// Task Function Prototypes
void flight_controller(void);
void process_remote_data(RemoteSetPoints_t* setPoints, uint16_t* payload);
void update_escs(uint16_t throttle, double pitchPID, double rollPID, double yawPID);
double pid_update(PID_t* pid, double error, uint64_t now);
void remote_data_return_init(int periodUS);
void remote_data_callback(void* args);

#endif
