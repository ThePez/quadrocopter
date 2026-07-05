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
    double kp;       // Proportional scaler
    double ki;       // Intergral scaler
    double kd;       // Derivative scaler
    double intLimit; // Intergral limiting term
    double divLimit; // Derivative limiting term
    double dt;       // Time delta
};

struct pid_result_t {
    double proportional; // Current "P" Output
    double derivative;   // Current "D" Output
    double intergral;    // Current "I" Output (is a running total)
    double error;        // Previous error
};

void pid_init_params(struct pid_parameters_t* params, double kp, double kd, double ki);
double pid_update(struct pid_parameters_t* params, struct pid_result_t* values, double ref,
                  double actual);
void pid_reset(struct pid_result_t* pid);

#endif
