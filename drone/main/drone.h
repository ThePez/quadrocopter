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

#define MAX_RATE 200.0
#define MAX_ANGLE 30.0

#define MIN_THROTTLE 1000.0
#define MAX_THROTTLE 2000.0

///////////////////////////// Structures & Enums /////////////////////////////

typedef enum { PID_MODIFIERS, SETPOINT_UPDATE } MessageType_t;

typedef enum { FLIGHT_MODE_RATE, FLIGHT_MODE_ANGLE } FlightMode_t;

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
    double intLimit;     // Intergral limiting term
    uint64_t prevTimeUS; // Last update time in us
} PID_t;

typedef struct {
    double pitchPID;
    double rollPID;
    double yawPID;
} PIDOutputs_t;

typedef struct {
    uint16_t motorA;
    uint16_t motorB;
    uint16_t motorC;
    uint16_t motorD;
} MotorPeriods_t;

typedef struct {
    RemoteSetPoints_t* setPoints;
    Telemitry_t* imuData;
    PIDOutputs_t* motorInputs;
    MotorPeriods_t* motorOutputs;
    FlightMode_t mode;
    uint8_t failsafeActive;
    uint8_t connectedToRemote;
    uint16_t battery;
} BlackBox_t;

///////////////////////////////// Prototypes /////////////////////////////////

/**
 * @brief Switch between flight control modes and reset PID controllers.
 *
 * Changes the active flight mode and performs necessary PID controller resets
 * to prevent integrator windup when transitioning between modes. Rate PID
 * controllers are always reset, while angle PID controllers are only reset
 * when switching to angle mode.
 *
 * @param mode The target flight mode (FLIGHT_MODE_ANGLE or FLIGHT_MODE_RATE).
 *
 * @note Only performs mode switch and resets if the new mode differs from current.
 * @note Rate PIDs (pitch, roll, yaw) are reset on every mode change.
 * @note Angle PIDs (pitch, roll) are reset only when entering angle mode.
 * @note Logs the mode change for debugging and monitoring purposes.
 */
void set_flight_mode(FlightMode_t mode, BlackBox_t* box);

/**
 * @brief Main PID control loop for the quadcopter.
 *
 * Waits for new telemetry (IMU) or radio control data using a FreeRTOS QueueSet.
 * Supports both rate and angle control modes with cascaded PID loops.
 * - Rate mode: Direct stick-to-rate control for acrobatic flying
 * - Angle mode: Stick-to-angle control with automatic leveling
 *
 * Maintains stable flight by adjusting motor speeds based on sensor and user input.
 *
 * @note Must be run as a FreeRTOS task.
 * @note Requires radioReceiverQueue, radioTransmitterQueue and imuQueue to be created before running.
 */
void flight_controller(void* pvParams);

void failsafe(BlackBox_t* box);

void process_positional_data(BlackBox_t* box);

/**
 * @brief Decodes a payload of radio data into control setpoints or PID tuning values.
 *
 * Interprets incoming data as either setpoint updates or PID modifier packets.
 * Updates global PID parameters or control targets based on message type.
 *
 * @param setPoints Pointer to the control structure to be updated.
 * @param payload   Pointer to a (16-word or 32-byte) array received from the radio.
 */
void process_remote_data(BlackBox_t* box, uint16_t* payload);

/**
 * @brief Updates ESC PWM signals based on control inputs and PID outputs.
 *
 * Applies a mixing algorithm to compute motor-specific throttle values that
 * reflect the desired pitch, roll, and yaw behavior, and writes the result
 * to all four ESCs using esc_pwm_set_duty_cycle().
 *
 * If a notification from a remote data timer is pending, sends motor state
 * back via the radioTransmitterQueue.
 *
 * @param throttle Base throttle input (1000–2000 µs).
 * @param pitchPID Output from pitch PID controller.
 * @param rollPID  Output from roll PID controller.
 * @param yawPID   Output from yaw PID controller.
 */
void update_escs(BlackBox_t* box);

/**
 * @brief Performs a single PID control step.
 *
 * Computes the control signal based on the current error, elapsed time,
 * and PID configuration parameters (kp, ki, kd).
 *
 * @param pid   Pointer to the PID_t structure.
 * @param error Current error value.
 * @param now   Current time in microseconds.
 * @return      Control output from the PID calculation.
 *
 * @note Updates the internal state of the PID structure (integral, derivative, etc.).
 */
double pid_update(PID_t* pid, double error, uint64_t now);

/**
 * @brief Resets the PID controller's internal state.
 *
 * Clears the integral accumulator and previous error to prevent
 * windup when switching modes or reinitializing control loops.
 *
 * @param pid Pointer to the PID structure to reset
 */
void pid_reset(PID_t* pid);

/**
 * @brief Initializes a periodic timer to signal that remote data can be returned.
 *
 * Creates and starts an ESP timer that periodically triggers a notification
 * to the flight controller task, allowing it to send motor state over radio.
 *
 * @param periodUS The timer period in microseconds.
 */
void timer_task_callback_init(int periodUS, BlackBox_t* box, void (*cb)(void*));

void battery_callback(void* args);

/**
 * @brief Timer callback to notify the flight controller for radio data transmission.
 *
 * Sends a direct task notification to the flight controller to allow it
 * to transmit updated motor state back to the remote controller.
 *
 * @param args Unused (can be NULL).
 */
void remote_data_callback(void* args);

#endif
