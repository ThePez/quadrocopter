/*
 *****************************************************************************
 * File: pid.c
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#include "pid.h"

#include "common_functions.h"

void pid_init_params(struct pid_parameters_t *params, float kp, float kd, float ki,
                     float dtermAlpha)
{
    params->divLimit = PID_DIV_LIMIT;
    params->intLimit = PID_INT_LIMIT;
    params->kp = kp;
    params->kd = kd;
    params->ki = ki;
    params->dtermAlpha = dtermAlpha;
}

float pid_update(const struct pid_parameters_t *params, struct pid_result_t *values, float ref,
                 float actual, float dt)
{
    // Calculate Error
    float error = ref - actual;

    // P - Term
    values->proportional = params->kp * error;

    // I - Term
    values->intergral = constrainf(values->intergral + params->ki * error * dt, -params->intLimit,
                                   params->intLimit);

    // D (on measurement, so setpoint steps don't kick it)
    if (values->primed) {
        float rawDerivative = constrainf(-params->kd * (actual - values->lastMeasurement) / dt,
                                         -params->divLimit, params->divLimit);
        values->derivative += params->dtermAlpha * (rawDerivative - values->derivative);
    }

    // Update state
    values->lastMeasurement = actual;
    values->primed = 1;

    return values->proportional + values->intergral + values->derivative;
}

void pid_reset(struct pid_result_t *pid)
{
    if (pid) {
        pid->proportional = 0.0f;
        pid->intergral = 0.0f;
        pid->derivative = 0.0f;
        pid->lastMeasurement = 0.0f;
        pid->primed = 0;
    }
}
