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
 * @brief Initialize the joysticks module and create the sampling task.
 *
 * Sets up the MCP3208 ADC, configures the mode switch interrupt, and creates
 * the joysticks sampling task.
 *
 * @param spiMutex Pointer to SPI mutex for thread-safe ADC access.
 * @param spiHost SPI host device identifier for MCP3208 communication.
 *
 * @return ESP_OK on success, ESP_FAIL on initialization failure.
 *
 * @note Creates a FreeRTOS task with JOYSTICKS_PRIORITY and JOYSTICKS_STACK size.
 * @note Allocates memory for task parameters that must be freed by the task.
 */
esp_err_t joysticks_module_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost);

/**
 * @brief Configure GPIO interrupt for mode switching button.
 *
 * Sets up GPIO pin 33 as input with pull-up enabled and configures interrupt
 * service to trigger on both rising and falling edges. Installs the provided
 * handler function to process button press events.
 *
 * @param handler Pointer to interrupt handler function.
 *
 * @return ESP_OK on success, appropriate ESP error code on failure.
 *
 * @note Uses internal pull-up resistor assuming active-low button configuration.
 * @note Installs ISR service if not already present (handles ESP_ERR_INVALID_STATE).
 */
esp_err_t mode_swap_interrupt_init(void* handler);

/**
 * @brief FreeRTOS task for sampling joystick inputs and handling mode switching.
 *
 * Continuously reads four joystick axis values from MCP3208 ADC channels 0-3
 * and monitors for mode switch notifications from the interrupt handler.
 * Maintains a mode flag (angle/rate) and sends sampled data plus mode state
 * to the joysticksQueue for downstream processing.
 *
 * @param pvParams Pointer to adcInputParams_t containing the SPI mutex.
 *
 * @note Uses SPI-protected access with mutex for thread safety.
 * @note Creates joysticksQueue if not already present during initialization.
 * @note Mode flag: 1 = Angle mode, 0 = Rate mode (stored in adcValues[4]).
 * @note Designed to run indefinitely as a FreeRTOS task.
 */
void joysticks_task(void* pvParams);

// Task Handle
extern TaskHandle_t joysticksTaskHandle;

// Queue Handle
extern QueueHandle_t joysticksQueue;

#endif
