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

static void adc_input_task(void* pvParams);

TaskHandle_t adcInputTask = NULL;
QueueHandle_t adcInputQueue = NULL;

static const char* TAG = "JOYSTICK";

/* adc_task_init ()
 * ----------------
 *
 */
void adc_task_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost) {

    adcInputParams_t* params = pvPortMalloc(sizeof(adcInputParams_t*));
    params->host = spiHost; // VSPI_HOST;
    params->spiMutex = spiMutex;
    xTaskCreate(&adc_input_task, "ADC_INPUTS", ADC_STACK, (void*) params, ADC_PRIORITY, &adcInputTask);
}

/* adc_input_task()
 * --------------------
 * Continuously reads analog joystick and slider inputs using the MCP3208 ADC.
 *
 * Initializes the SPI bus for the ADC and reads five channels:
 *   - Channels 1–4: Joystick axes
 *   - Channel 0: Slider or other analog input
 *
 * Collected data is sent to the remote_controller task via a queue.
 *
 * Runs continuously as a FreeRTOS task.
 */
static void adc_input_task(void* pvParams) {

    adcInputParams_t* params = (adcInputParams_t*) pvParams;
    SemaphoreHandle_t spiMutex = *(params->spiMutex);
    spi_host_device_t host = params->host;

    uint16_t adcValues[5];
    while (!adcInputQueue) {
        // Loop to ensure the input queue is created
        adcInputQueue = xQueueCreate(5, sizeof(adcValues));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    xSemaphoreTake(spiMutex, portMAX_DELAY);
    mcp3208_init(host);
    xSemaphoreGive(spiMutex);

    ESP_LOGI(TAG, "Task initialised");

    while (1) {

        if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(1)) == pdTRUE) {

            for (uint8_t i = 0; i < 5; i++) {
                // Channels 1 to 4 are joysticks, 0 is slider
                adcValues[i] = mcp3208_read_adc_channel(i, MCP3208_SINGLE);
            }

            // Return spi mutex
            xSemaphoreGive(spiMutex);

            if (adcInputQueue) {
                xQueueSendToBack(adcInputQueue, adcValues, pdMS_TO_TICKS(5));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
