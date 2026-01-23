/*
 *****************************************************************************
 * File: drone.h
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
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

#define MAX_RATE 200.0 // deg/s
#define MAX_ANGLE 25.0 // deg
#define FAIL_ANGLE 30.0

#define MIN_THROTTLE 1000.0 // us
#define MAX_THROTTLE 2000.0 // us

#define REMOTE_UPDATE 1
#define PID_UPDATE 2

#define FAILSAFE_TIMEOUT_US 1000000 // 1 second
#define PID_LOOP_FREQ 2500          // 400 Hz -> 2.5ms -> 2500us
#define PID_INT_LIMIT 50
#define PID_DIV_LIMIT 50

///////////////////////////// Structures & Enums /////////////////////////////

typedef enum { ACRO, STABILISE } FlightMode_t; // ACRO is rate control (FPV mode), STABILISE is standard mode

typedef struct __packed {
    double pitch;
    double roll;
    double yaw;
    double throttle;
} TargetParameters_t;

typedef struct __packed {
    double kp;       // Proportional scaler
    double ki;       // Intergral scaler
    double kd;       // Derivative scaler
    double intLimit; // Intergral limiting term
    double divLimit; // Derivative limiting term
    double dt;       // Time delta
} PIDParameters_t;

typedef struct __packed {
    uint16_t command_id;
    uint16_t axis; // 0=Pitch, 1=Roll, 2=Yaw
    uint16_t mode; // 0=Rate, 1=Angle
    float kp;
    float ki;
    float kd;
    uint16_t reserved[9];
} pid_config_packet_t;

typedef struct __packed {
    double proportional; // Current "P" Output
    double derivative;   // Current "D" Output
    double intergral;    // Current "I" Output (is a running total)
    double prevError;    // Previous error
} PIDResult_t;

typedef struct __packed {
    PIDResult_t* pitch;
    PIDResult_t* roll;
    PIDResult_t* yaw;
} PIDFeedback_t;

typedef struct __packed {
    double pitch_pid;
    double roll_pid;
    double yaw_pid;
} PIDFinal_t;

typedef struct __packed {
    uint16_t motorA;
    uint16_t motorB;
    uint16_t motorC;
    uint16_t motorD;
} PWM_t;

typedef struct __packed {
    FlightMode_t mode;
    uint8_t armed;
    uint8_t comms_failsafe_active;
    uint8_t angle_failsafe_active;
    uint16_t battery;
    uint64_t lastRemoteTime;
} DroneConfig_t;

///////////////////////////////// Prototypes /////////////////////////////////

void input_control(void* pvParams);
void pid_control(void* pvParams);
void set_flight_mode(FlightMode_t mode);
void motor_shutdown(void);
void angle_failsafe(void);
void comms_failsafe(void);
void update_escs(void);

#endif
