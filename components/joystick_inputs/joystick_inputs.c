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
#include "esp_timer.h"
#include "mcp3208.h"

#define CHECK_ERR(code, msg)                                                                                           \
    do {                                                                                                               \
        esp_err_t err = (code);                                                                                        \
        if (err != ESP_OK) {                                                                                           \
            ESP_LOGE(TAG, msg);                                                                                        \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

TaskHandle_t joysticksTaskHandle = NULL;
QueueHandle_t joysticksQueue = NULL;

static const char* TAG = "JOYSTICK";
static uint8_t pushAllowed = 1;
static uint64_t prevFalling = 0;
static uint64_t prevRising = 0;

/**
 * @brief GPIO interrupt handler for mode switching button.
 *
 * Handles both falling and rising edge interrupts on GPIO pin 33 to implement
 * button debouncing and mode switching functionality. On falling edge (button press),
 * notifies the joysticks task to toggle between angle and rate modes.
 *
 * @note This function runs in interrupt context (IRAM_ATTR).
 * @note Uses 500us (50ms) debouncing to prevent multiple triggers.
 * @note Falling edge triggers mode change; rising edge re-enables push detection.
 */
static void IRAM_ATTR mode_handler(void) {

    gpio_num_t pin = 33;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int state = gpio_get_level(pin);
    uint32_t currentTick = esp_timer_get_time();
    if (!state && pushAllowed && (currentTick - prevRising >= 500)) {

        // Falling edge
        prevFalling = currentTick;
        pushAllowed = 0;
        vTaskNotifyGiveFromISR(joysticksTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    } else if (state && (currentTick - prevFalling >= 500)) {

        // Rising edge
        prevRising = currentTick;
        pushAllowed = 1;
    }
}

esp_err_t joysticks_module_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost) {

    xSemaphoreTake(*spiMutex, portMAX_DELAY);
    mcp3208_init(spiHost); // VSPI_HOST;
    xSemaphoreGive(*spiMutex);

    mode_swap_interrupt_init((void*) &mode_handler);

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

esp_err_t mode_swap_interrupt_init(void* handler) {

    gpio_num_t pin = 33;
    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT,               // Direction of the pin
        .pin_bit_mask = (1ULL << pin),         // Pin is listed here as a bit mask
        .pull_up_en = GPIO_PULLUP_ENABLE,      // No internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // No internal pull-down
        .intr_type = GPIO_INTR_ANYEDGE         // Interrupt on both edges, switch is active low
    };

    CHECK_ERR(gpio_config(&config), "GPIO config failed");
    CHECK_ERR(gpio_intr_disable(pin), "Interrupt disable failed");
    // Install the ISR service if not already installed
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ISR install service failed");
        return err;
    }

    // Hook ISR handler for the IQR pin
    CHECK_ERR(gpio_isr_handler_add(pin, handler, NULL), "ISR handler add failed");
    return ESP_OK;
}

void joysticks_task(void* pvParams) {

    adcInputParams_t* params = (adcInputParams_t*) pvParams;
    SemaphoreHandle_t spiMutex = *(params->spiMutex);

    uint16_t adcValues[5];
    adcValues[4] = 1; // 1 => Angle mode, 0 => Rate mode
    while (!joysticksQueue) {
        // Loop to ensure the input queue is created
        joysticksQueue = xQueueCreate(JOYSTICKS_QUEUE_LENGTH, sizeof(adcValues));
        vTaskDelay(JOYSTICKS_DELAY);
    }

    ESP_LOGI(TAG, "Task initialised");

    while (1) {

        if (xSemaphoreTake(spiMutex, JOYSTICKS_DELAY) == pdTRUE) {
            for (uint8_t i = 0; i < 4; i++) {
                // Channels 1 to 4 are joysticks, 0 is slider
                adcValues[i] = mcp3208_read_adc_channel(i, MCP3208_SINGLE);
            }

            if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
                adcValues[4] ^= 1; // Toggle the value
                ESP_LOGI(TAG, "ISR Triggered");
            }

            // Return spi mutex
            xSemaphoreGive(spiMutex);
            // Send recieved data to the queue for processing
            xQueueSendToBack(joysticksQueue, adcValues, JOYSTICKS_DELAY);
        }

        vTaskDelay(JOYSTICKS_DELAY);
    }
}
