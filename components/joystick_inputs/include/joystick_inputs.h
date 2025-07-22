/*
 *****************************************************************************
 * File: joystick_inputs.h
 * Author: Jack Cairns
 * Date: 23-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef JOYSTICK_INPUTS_H
#define JOYSTICK_INPUTS_H

#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "mcp3208.h"

#define ADC_STACK (configMINIMAL_STACK_SIZE * 2)
#define ADC_PRIORITY (tskIDLE_PRIORITY + 4)

typedef struct {
    SemaphoreHandle_t* spiMutex;
    spi_host_device_t host;
} adcInputParams_t;

void adc_task_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost);

extern TaskHandle_t adcInputTask;
extern QueueHandle_t adcInputQueue;

#endif
