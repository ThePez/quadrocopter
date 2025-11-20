/*
 ******************************************************************************
 * File: remote.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 ******************************************************************************
 */

#include "remote.h"

#include "common_functions.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mcp3208.h"
#include "nrf24l01plus.h"

//////////////////////////// Function Prototypes /////////////////////////////

/**
 * @brief FreeRTOS task that collect and transmits control data.
 *
 * Waits for incoming joystick ADC data from the joystick input queue,
 * forms a control packet, and sends it to the radio transmitter queue.
 * It can also receive and print feedback from the radio receiver queue.
 *
 * Behavior:
 * - Waits until all required queues (joystick, transmitter, receiver) are ready.
 * - Receives 5 ADC values (throttle, pitch, roll, yaw, slider) from `joysticksQueue`.
 * - Builds a 16-element packet:
 *     - Index 0: Command ID (currently fixed to 1 for "setpoint update").
 *     - Index 1–5: Control values. These are currently manually overridden.
 *     - Index 6-15: '0', Filler values.
 * - Sends the packet to `radioTransmitterQueue` for NRF24L01+ transmission.
 * - Optionally receives and prints debug values from `radioReceiverQueue`.
 *
 * Notes:
 * - ADC values received from joystick task are replaced by hardcoded midpoint values (2048),
 *   likely for testing.
 *
 * This task runs indefinitely.
 */
static void remote_controller(void);

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
static void intr_handler(void* args);

/**
 * @brief Configure GPIO interrupt for mode switching button.
 *
 * Sets up GPIO pin 33 as input with pull-up enabled and configures interrupt
 * service to trigger on both rising and falling edges. Installs the provided
 * handler function to process button press events.
 *
 * @return ESP_OK on success, appropriate ESP error code on failure.
 *
 * @note Uses internal pull-up resistor assuming active-low button configuration.
 * @note Installs ISR service if not already present (handles ESP_ERR_INVALID_STATE).
 */
static esp_err_t mode_swap_interrupt_init(void);

static esp_err_t emergancy_interrupt_init(void);

////////////////////////////// Global Variables //////////////////////////////

// Handle for the remote_controller FreeRTOS task
TaskHandle_t remoteTaskHandle = NULL;
// Button states
button_state_t mode_button = {.pin = 33, .pushAllowed = 1};
button_state_t emergency_button = {.pin = 32, .pushAllowed = 1};
// Logging tag
#define TAG "REMOTE"

//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Main entry point for the remote controller firmware.
 *
 * Sets up SPI buses and initializes the radio and joystick modules.
 * Creates and launches the main `remote_controller` task.
 */
void app_main(void) {
    spi_bus_setup(VSPI_HOST);
    spi_bus_setup(HSPI_HOST);

    radio_module_init(&spiHMutex, HSPI_HOST);
    mcpx_task_init(&spiVMutex, 0x1F, VSPI_HOST);

    mode_swap_interrupt_init();
    emergancy_interrupt_init();
    xTaskCreate((void*) &remote_controller, "REMOTE_TASK", REMOTE_STACK, NULL, REMOTE_PRIO, &remoteTaskHandle);
}

static void remote_controller(void) {

    uint16_t adcValues[5] = {0};
    uint16_t adcPacket[16] = {0};

    // Idle until all queue's are created
    while (!mcpxQueue || !radioTransmitterQueue || !radioReceiverQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    uint8_t flightMode = 0; // 0 Rate, 1 Angle
    uint8_t modePressed = 0;
    uint8_t emergencyPressed = 0;
    uint8_t emergencyShutdown = 0;
    uint8_t armed = 0;
    uint32_t notifyValue;
    ESP_LOGW(TAG, "Waiting to be armed, press both joysticks.");

    while (1) {
        // Get adc values from the input queue
        if (xQueueReceive(mcpxQueue, adcValues, REMOTE_DELAY) == pdTRUE) {

            // Was a joystick pressed?
            if (xTaskNotifyWait(0, 0xFFFFFFFF, &notifyValue, 0) == pdTRUE) {
                switch (notifyValue) {
                case 33: // Flight Mode
                    modePressed = 1;
                    if (armed) {
                        flightMode ^= 1;
                        ESP_LOGI(TAG, "Flight Mode %d", flightMode);
                    }
                    break;

                case 32: // Emergancy motor shutoff
                    emergencyPressed = 1;
                    if (armed) {
                        emergencyPressed = 0;
                        emergencyShutdown = 1;
                        modePressed = 0;
                        armed = 0;
                        ESP_LOGE(TAG, "Emergancy");
                    }

                    break;
                }
            }

            // Check if now armed
            if (!armed && modePressed && emergencyPressed) {
                if (!emergencyShutdown) {
                    armed = 1;
                    ESP_LOGW(TAG, "REMOTE ARMED — communication is now enabled.");
                } else if (adcValues[0] < 50) {
                    emergencyShutdown = 0;
                    armed = 1;
                    ESP_LOGW(TAG, "REMOTE ARMED — communication is now enabled.");
                }
            }

            memset(adcPacket, 0, sizeof(adcPacket));
            adcPacket[0] = 1;                           // Setpoint update
            adcPacket[1] = (!armed) ? 0 : adcValues[0]; // Throttle
            adcPacket[2] = adcValues[1];                // Pitch
            adcPacket[3] = adcValues[2];                // Roll
            adcPacket[4] = adcValues[3];                // Yaw
            adcPacket[5] = (uint16_t) flightMode;       // Mode

            // Send the resulting packet to the radio task
            if (!armed) {
                continue;
            }

            if (radioTransmitterQueue) {
                xQueueSendToFront(radioTransmitterQueue, adcPacket, pdMS_TO_TICKS(5));
            }
        }

        // Was data recieved back from the drone? (No waiting)
        if (xQueueReceive(radioReceiverQueue, adcPacket, 0) == pdTRUE) {
            int16_t* data = (int16_t*) adcPacket;
            // If not armed don't print data
            if (!armed) {
                continue;
            }

            ESP_LOGI(TAG,
                     "Mode: %s, Angles: P=%d R=%d Y=%d, Rates: P=%d R=%d Y=%d, PID: P=%d R=%d Y=%d, Motors: FL=%u "
                     "BL=%u BR=%u FR=%u",
                     data[6] ? "ANGLE" : "RATE", data[0], data[1], data[2], data[3], data[4], data[5], data[7], data[8],
                     data[9], adcPacket[10], adcPacket[11], adcPacket[12], adcPacket[13]);
        }
    }
}

static void IRAM_ATTR intr_handler(void* args) {

    button_state_t* button = (button_state_t*) args;
    gpio_num_t pin = button->pin;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int state = gpio_get_level(pin);
    uint64_t currentTick = esp_timer_get_time();
    if (!state && button->pushAllowed && (currentTick - button->prevRising >= 50000)) {

        // Falling edge
        button->prevFalling = currentTick;
        button->pushAllowed = 0;
        xTaskNotifyFromISR(remoteTaskHandle, button->pin, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    } else if (state && (currentTick - button->prevFalling >= 50000)) {

        // Rising edge
        button->prevRising = currentTick;
        button->pushAllowed = 1;
    }
}

static esp_err_t interrupt_init(void* handler, button_state_t* button) {

    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT,               // Direction of the pin
        .pin_bit_mask = (1ULL << button->pin), // Pin is listed here as a bit mask
        .pull_up_en = GPIO_PULLUP_ENABLE,      // No internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // No internal pull-down
        .intr_type = GPIO_INTR_ANYEDGE         // Interrupt on both edges, switch is active low
    };

    CHECK_ERR(gpio_config(&config), "GPIO config failed");
    CHECK_ERR(gpio_intr_disable(button->pin), "Interrupt disable failed");
    // Install the ISR service if not already installed
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ISR install service failed");
        return err;
    }

    // Hook ISR handler for the IQR pin
    CHECK_ERR(gpio_isr_handler_add(button->pin, handler, button), "ISR handler add failed");
    return ESP_OK;
}

static esp_err_t emergancy_interrupt_init(void) {
    return interrupt_init((void*) &intr_handler, &emergency_button);
}

static esp_err_t mode_swap_interrupt_init(void) {
    return interrupt_init((void*) &intr_handler, &mode_button);
}
