/*
 *****************************************************************************
 * File: pid.h
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

#define PID_INT_LIMIT 50.0f
#define PID_DIV_LIMIT 50.0f

struct pid_parameters_t {
    float kp;         // Proportional scaler
    float ki;         // Intergral scaler
    float kd;         // Derivative scaler
    float intLimit;   // Intergral limiting term
    float divLimit;   // Derivative limiting term
    float dtermAlpha; // D-term low-pass filter coefficient (1 = unfiltered)
};

struct pid_result_t {
    float proportional;    // Current "P" Output
    float derivative;      // Current "D" Output (low-pass filtered)
    float intergral;       // Current "I" Output (is a running total)
    float lastMeasurement; // Previous "actual" value, for derivative-on-measurement
    uint8_t primed;        // False until lastMeasurement holds a real sample
};

/**
 * @brief Sets up a PID controller's gains and limits.
 *
 * The integral/derivative clamp limits are taken from
 * PID_INT_LIMIT/PID_DIV_LIMIT.
 *
 * @param params     Parameter struct to initialize.
 * @param kp         Proportional gain.
 * @param kd         Derivative gain.
 * @param ki         Integral gain.
 * @param dtermAlpha D-term low-pass filter coefficient (1 = unfiltered, lower towards 0 to smooth).
 */
void pid_init_params(struct pid_parameters_t *params, float kp, float kd, float ki,
                     float dtermAlpha);

/**
 * @brief Runs one iteration of the PID loop and returns the control output.
 *
 * Computes proportional, clamped integral, and low-pass-filtered
 * derivative-on-measurement terms, updates values in place (including
 * lastMeasurement for the next call), and returns their sum. The derivative
 * is skipped on the first call after pid_reset(), since there is no previous
 * measurement to difference against.
 *
 * @param params Fixed gains/limits for this controller.
 * @param values Running state for this controller; updated in place.
 * @param ref    Setpoint (reference) value.
 * @param actual Current measured value.
 * @param dt     Time since the previous call, in seconds (must be > 0).
 * @return Combined P+I+D control output.
 */
float pid_update(const struct pid_parameters_t *params, struct pid_result_t *values, float ref,
                 float actual, float dt);

/**
 * @brief Zeroes a PID controller's running state (P/I/D terms and last measurement).
 *
 * Used when switching flight modes, updating gains, or sitting on the ground,
 * to avoid stale integrator/derivative state carrying over.
 *
 * @param pid State to reset; no-op if NULL.
 */
void pid_reset(struct pid_result_t *pid);

#endif
