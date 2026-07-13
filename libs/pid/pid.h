/*
 *****************************************************************************
 * File: pid.h
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#ifndef PID_H
#define PID_H

#define PID_LOOP_FREQ 2500 // 400 Hz -> 2.5ms -> 2500us
#define PID_INT_LIMIT 50
#define PID_DIV_LIMIT 50

struct pid_parameters_t {
    double kp;         // Proportional scaler
    double ki;         // Intergral scaler
    double kd;         // Derivative scaler
    double intLimit;   // Intergral limiting term
    double divLimit;   // Derivative limiting term
    double dt;         // Time delta
    double dtermAlpha; // D-term low-pass filter coefficient (1 = unfiltered)
};

struct pid_result_t {
    double proportional;    // Current "P" Output
    double derivative;      // Current "D" Output
    double intergral;       // Current "I" Output (is a running total)
    double lastMeasurement; // Previous "actual" value, for derivative-on-measurement
    double dtermFiltered;   // Low-pass filtered D-term state
};

/**
 * @brief Sets up a PID controller's gains and fixed loop parameters.
 *
 * Loop period (dt), and the integral/derivative clamp limits are taken from
 * PID_LOOP_FREQ/PID_INT_LIMIT/PID_DIV_LIMIT. dtermAlpha defaults to 1.0
 * (unfiltered D-term).
 *
 * @param params Parameter struct to initialize.
 * @param kp     Proportional gain.
 * @param kd     Derivative gain.
 * @param ki     Integral gain.
 */
void pid_init_params(struct pid_parameters_t* params, double kp, double kd, double ki);

/**
 * @brief Runs one iteration of the PID loop and returns the control output.
 *
 * Computes proportional, clamped integral, and low-pass-filtered
 * derivative-on-measurement terms, updates values in place (including
 * lastMeasurement for the next call), and returns their sum.
 *
 * @param params Fixed gains/limits for this controller.
 * @param values Running state for this controller; updated in place.
 * @param ref    Setpoint (reference) value.
 * @param actual Current measured value.
 * @return Combined P+I+D control output.
 */
double pid_update(struct pid_parameters_t* params, struct pid_result_t* values, double ref,
                  double actual);

/**
 * @brief Zeroes a PID controller's running state (P/I/D terms and last measurement).
 *
 * Used when switching flight modes or updating gains, to avoid stale
 * integrator/derivative state carrying over.
 *
 * @param pid State to reset; no-op if NULL.
 */
void pid_reset(struct pid_result_t* pid);

#endif
