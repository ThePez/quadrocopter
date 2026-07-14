/*
 *****************************************************************************
 * File: pid.c
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#include "pid.h"

#include "common_functions.h"

void pid_init_params(struct pid_parameters_t* params, double kp, double kd, double ki,
                     double dtermAlpha) {
    params->divLimit = PID_DIV_LIMIT;
    params->intLimit = PID_INT_LIMIT;
    params->dt = PID_LOOP_FREQ;
    params->kp = kp;
    params->kd = kd;
    params->ki = ki;
    params->dtermAlpha = dtermAlpha;
}

double pid_update(struct pid_parameters_t* params, struct pid_result_t* values, double ref,
                  double actual) {
    double dt = params->dt * 1e-6;
    double error = ref - actual;
    // P
    values->proportional = params->kp * error;
    // I
    values->intergral = constrainf(values->intergral + params->ki * error * dt, -params->intLimit,
                                   params->intLimit);
    // D
    double rawDerivative = constrainf(-params->kd * (actual - values->lastMeasurement) / dt,
                                      -params->divLimit, params->divLimit);
    values->dtermFiltered =
        (params->dtermAlpha * rawDerivative) + ((1 - params->dtermAlpha) * values->dtermFiltered);
    values->derivative = values->dtermFiltered;

    // Update state
    values->lastMeasurement = actual;

    return values->proportional + values->intergral + values->derivative;
}

void pid_reset(struct pid_result_t* pid) {
    if (pid) {
        pid->proportional = 0.0;
        pid->intergral = 0.0;
        pid->derivative = 0.0;
        pid->lastMeasurement = 0.0;
        pid->dtermFiltered = 0.0;
    }
}
