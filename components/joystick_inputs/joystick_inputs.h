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

// STD C lib headers
#include <stdint.h>

// ESP-IDF Prebuilts
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define JOYSTICKS_STACK (configMINIMAL_STACK_SIZE * 2)
#define JOYSTICKS_PRIORITY (tskIDLE_PRIORITY + 4)
#define JOYSTICKS_QUEUE_LENGTH 5
#define JOYSTICKS_DELAY (pdMS_TO_TICKS(50))

// Input struct for setting up the joystick module
typedef struct {
    SemaphoreHandle_t* spiMutex;
} adcInputParams_t;

/**
 * @brief Initializes the joystick input module.
 *
 * Sets up the MCP3208 ADC over SPI and starts a FreeRTOS task
 * to continuously read joystick and slider inputs.
 *
 * @param spiMutex Pointer to a mutex protecting SPI access.
 * @param spiHost  SPI host connected to the MCP3208 chip.
 */
esp_err_t joysticks_module_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost);

// Task Handle
extern TaskHandle_t joysticksTaskHandle;

// Queue Handle
extern QueueHandle_t joysticksQueue;

#endif
