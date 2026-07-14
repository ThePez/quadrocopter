/*
 *****************************************************************************
 * File: drone_defaults.h
 * Author: Jack Cairns
 * Date: 15-07-2026
 *****************************************************************************
 */

#ifndef DRONE_DEFAULTS_H
#define DRONE_DEFAULTS_H

#define MAX_RATE 200.0 // deg/s
#define MAX_ANGLE 25.0 // deg
#define FAIL_ANGLE 30.0

#define MIN_THROTTLE 1000.0 // us
#define MAX_THROTTLE 2000.0 // us

#define FAILSAFE_TIMEOUT_US 2000000 // 1 second

#define LOW_VOLTAGE 14000      // mV (Warn operator at this voltage)
#define CRITICAL_VOLTAGE 13600 // mV (min voltage the battery can be)

#endif
