/*
 *****************************************************************************
 * File: remote_defaults.h
 * Author: Jack Cairns
 * Date: 15-07-2026
 *****************************************************************************
 */

#ifndef REMOTE_DEFAULTS_H
#define REMOTE_DEFAULTS_H

// Identity calibration (no correction) - until a real calibration routine
// records each joystick's actual min/centre/max, raw ADC values should pass
// through unchanged.
#define JOYSTICK_CAL_MIN 0
#define JOYSTICK_CAL_CENTRE 2048
#define JOYSTICK_CAL_MAX 4095

#define VOLTAGE_CAL_MULTIPLIER 1.0

// Placeholders - the remote has no battery-sensing hardware yet.
#define LOW_VOLTAGE 0
#define CRITICAL_VOLTAGE 0

#endif
