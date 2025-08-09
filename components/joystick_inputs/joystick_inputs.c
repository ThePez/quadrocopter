/*
 *****************************************************************************
 * File: joystick_inputs.c
 * Author: Jack Cairns
 * Date: 13-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "joystick_inputs.h"
#include "esp_log.h"
#include "mcp3208.h"

static void joysticks_task(void* pvParams);

TaskHandle_t joysticksTaskHandle = NULL;
QueueHandle_t joysticksQueue = NULL;

static const char* TAG = "JOYSTICK";

esp_err_t joysticks_module_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost) {

    xSemaphoreTake(*spiMutex, portMAX_DELAY);
    mcp3208_init(spiHost); // VSPI_HOST;
    xSemaphoreGive(*spiMutex);

    adcInputParams_t* params = pvPortMalloc(sizeof(adcInputParams_t*));
    if (params == NULL) {
        // malloc fails
        ESP_LOGE(TAG, "Paramater setup failed");
        return ESP_FAIL;
    }
    params->spiMutex = spiMutex;
    BaseType_t err = xTaskCreate(&joysticks_task, "JOYSTICKS", JOYSTICKS_STACK, (void*) params, JOYSTICKS_PRIORITY,
                                 &joysticksTaskHandle);
    if (err != pdTRUE) {
        ESP_LOGE(TAG, "Task setup failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief FreeRTOS task for sampling joystick and slider inputs.
 *
 * Reads five analog channels from the MCP3208 ADC:
 *   - Channel 0: Slider or auxiliary analog input
 *   - Channels 1 to 4: Joystick axes
 *
 * Sampled values are sent to the joysticksQueue for downstream processing.
 *
 * @param pvParams Pointer to adcInputParams_t containing the SPI mutex.
 *
 * @note Uses SPI-protected access and MCP3208 driver functions.
 * @note Designed to run indefinitely as a FreeRTOS task.
 */
static void joysticks_task(void* pvParams) {

    adcInputParams_t* params = (adcInputParams_t*) pvParams;
    SemaphoreHandle_t spiMutex = *(params->spiMutex);

    uint16_t adcValues[5];
    while (!joysticksQueue) {
        // Loop to ensure the input queue is created
        joysticksQueue = xQueueCreate(JOYSTICKS_QUEUE_LENGTH, sizeof(adcValues));
        vTaskDelay(JOYSTICKS_DELAY);
    }

    ESP_LOGI(TAG, "Task initialised");

    while (1) {

        if (xSemaphoreTake(spiMutex, JOYSTICKS_DELAY) == pdTRUE) {
            for (uint8_t i = 0; i < 5; i++) {
                // Channels 1 to 4 are joysticks, 0 is slider
                adcValues[i] = mcp3208_read_adc_channel(i, MCP3208_SINGLE);
            }

            // Return spi mutex
            xSemaphoreGive(spiMutex);
            // Send recieved data to the queue for processing
            xQueueSendToBack(joysticksQueue, adcValues, JOYSTICKS_DELAY);
        }

        vTaskDelay(JOYSTICKS_DELAY);
    }
}
