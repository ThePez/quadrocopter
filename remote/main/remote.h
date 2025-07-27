/*
 *****************************************************************************
 * File: remote.h
 * Author: Jack Cairns
 * Date: 23-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef REMOTE_H
#define REMOTE_H

// STD C lib headers
#include <stdint.h>

// KConfig header
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define REMOTE_STACK (configMINIMAL_STACK_SIZE * 2)
#define REMOTE_PRIO (tskIDLE_PRIORITY + 2)

#define ADC_MIN 0
#define ADC_MAX 4095

#define MIN_RATE -200.0
#define MAX_RATE 200.0

#define MAX_ANGLE 30.0
#define MIN_ANGLE -30.0

#define MIN_THROTTLE 1000.0
#define MAX_THROTTLE 2000.0

void remote_controller(void);

#endif