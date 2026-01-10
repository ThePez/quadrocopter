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

#include "imu.h"

// STD C lib headers
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

// KConfig header
#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define SYS_STACK (configMINIMAL_STACK_SIZE * 2)
#define SYS_PRIO (tskIDLE_PRIORITY + 2)

#define ADC_MIN 0
#define ADC_MAX 4095

#define MAX_RATE 45.0  // deg/s
#define MAX_ANGLE 30.0 // deg

#define MIN_THROTTLE 1000.0 // us
#define MAX_THROTTLE 2000.0 // us

///////////////////////////// Structures & Enums /////////////////////////////

typedef enum { ACRO, STABILISE } FlightMode_t; // ACRO is rate control (FPV mode), STABILISE is standard mode

typedef struct {
    double pitch;
    double roll;
    double yaw;
    double throttle;
} TargetParameters_t;

typedef struct {
    double kp;       // Proportional scaler
    double ki;       // Intergral scaler
    double kd;       // Derivative scaler
    double intLimit; // Intergral limiting term
    double dt;       // Time delta
} PIDParameters_t;

typedef struct {
    double proportional; // Current "P" Output
    double derivative;   // Current "D" Output
    double intergral;    // Current "I" Output (is a running total)
    double prevError;    // Previous error
} PIDResult_t;

typedef struct {
    PIDResult_t* pitch;
    PIDResult_t* roll;
    PIDResult_t* yaw;
} PIDFeedback_t;

typedef struct {
    double pitch_pid;
    double roll_pid;
    double yaw_pid;
} PIDFinal_t;

typedef struct {
    uint16_t motorA;
    uint16_t motorB;
    uint16_t motorC;
    uint16_t motorD;
} PWM_t;

typedef struct {
    FlightMode_t mode;
    uint8_t armed;
    uint16_t battery;
    uint64_t lastRemoteTime;
} DroneConfig_t;

///////////////////////////////// Prototypes /////////////////////////////////

void sensor_control(void* pvParams);
void input_control(void* pvParams);
void pid_control(void* pvParams);
void memory_init(void);
void set_flight_mode(FlightMode_t mode);
void failsafe(void);
void update_escs(void);
double pid_update(PIDParameters_t* params, PIDResult_t* values, double ref, double actual);
void pid_reset(PIDResult_t* pid);
void timer_task_callback_init(int periodUS, void (*cb)(void*));
void pid_timer_init(void);
void pid_callback(void* args);
void battery_callback(void* args);
void remote_data_callback(void* args);

#endif
