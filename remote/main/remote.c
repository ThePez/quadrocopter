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
#include "espnow_comm.h"
#include "mcp3208.h"

//////////////////////////// Function Prototypes /////////////////////////////

static void remote_controller(void);
static void intr_handler(void* args);
static esp_err_t mode_swap_interrupt_init(void);
static esp_err_t emergancy_interrupt_init(void);

////////////////////////////// Global Variables //////////////////////////////

// Handle for the remote_controller FreeRTOS task
TaskHandle_t remoteTaskHandle = NULL;
// Button states
button_state_t mode_button = {.pin = MODE_BUTTON_PIN, .pushAllowed = 1};
button_state_t emergency_button = {.pin = SHUTOFF_BUTTON_PIN, .pushAllowed = 1};
// Logging tag
#define TAG "REMOTE"

#define STATUS_LED 2

//////////////////////////////////////////////////////////////////////////////

void app_main(void) {
    spi_bus_setup(VSPI_HOST);

    mcpx_task_init(&spiVMutex, 0x1F, VSPI_HOST, MCPx_CS_PIN_REMOTE); // Remote MCP3208 cs pin is 25

    uint8_t* macs[] = {drone_mac};
    esp_now_module_init(macs, 1);

    mode_swap_interrupt_init();
    emergancy_interrupt_init();
    xTaskCreate((void*) &remote_controller, "REMOTE_TASK", REMOTE_STACK, NULL, REMOTE_PRIO, &remoteTaskHandle);
}

static void remote_controller(void) {

    uint16_t adcValues[5] = {0};
    uint16_t adcPacket[16] = {0};

    // Idle until all queue's are created
    while (!mcpxQueue || !wifiQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    FlightMode_t flightMode = ACRO;
    uint8_t modePressed = 0;
    uint8_t emergencyPressed = 0;
    uint8_t emergencyShutdown = 0;
    uint8_t armed = 0;
    uint32_t notifyValue;

    // Initialize status LED
    gpio_config_t led_config = {.mode = GPIO_MODE_OUTPUT,
                                .pin_bit_mask = (1ULL << STATUS_LED),
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&led_config);
    gpio_set_level(STATUS_LED, 0); // Start with LED off for ACRO mode

    ESP_LOGW(TAG, "Waiting to be armed, press both joysticks.");

    while (1) {
        // Get adc values from the input queue
        if (xQueueReceive(mcpxQueue, adcValues, REMOTE_DELAY) == pdTRUE) {

            // Was a joystick pressed?
            if (xTaskNotifyWait(0, 0xFFFFFFFF, &notifyValue, 0) == pdTRUE) {
                switch (notifyValue) {
                case MODE_BUTTON_PIN: // Flight Mode
                    modePressed = 1;
                    if (armed) {
                        flightMode = (flightMode != ACRO) ? ACRO : STABILISE;
                        gpio_set_level(STATUS_LED, (flightMode == STABILISE) ? 1 : 0);
                        ESP_LOGI(TAG, "Flight Mode %s", (flightMode == STABILISE) ? "Stabilise" : "Acro");
                    }
                    break;

                case SHUTOFF_BUTTON_PIN: // Emergancy motor shutoff
                    emergencyPressed = 1;
                    if (armed) {
                        emergencyPressed = 0;
                        emergencyShutdown = 1;
                        modePressed = 0;
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
                } else if (adcValues[2] < 50) {
                    emergencyShutdown = 0;
                    armed = 1;
                    ESP_LOGW(TAG, "REMOTE REARMED — communication is now enabled.");
                }
            }

            memset(adcPacket, 0, sizeof(adcPacket));
            adcPacket[0] = 1;                                      // Setpoint update
            adcPacket[1] = (emergencyShutdown) ? 0 : adcValues[2]; // Throttle
            adcPacket[2] = adcValues[1];                           // Pitch
            adcPacket[3] = adcValues[0];                           // Roll
            adcPacket[4] = adcValues[3];                           // Yaw
            adcPacket[5] = (uint16_t) flightMode;                  // Mode

            // ESP_LOGI(TAG, "Throttle %d, Pitch %d, Roll %d, Yaw %d", adcValues[2], adcValues[1], adcValues[0],
            //          adcValues[3]);

            // Send the resulting packet to the drone
            if (armed) {
                esp_now_send(drone_mac, (uint8_t*) adcPacket, 32);
            }

            if (emergencyShutdown) {
                armed = 0;
            }
        }

        // Was data recieved back from the drone? (No waiting)
        // if (xQueueReceive(wifiQueue, adcPacket, 0) == pdTRUE) {
        //     int16_t* data = (int16_t*) adcPacket;
        //     // If not armed don't print data
        //     if (!armed) {
        //         continue;
        //     }

        //     ESP_LOGI(TAG,
        //              "Mode: %s, Angles: P=%d R=%d Y=%d, Rates: P=%d R=%d Y=%d, PID: P=%d R=%d Y=%d, Motors: FL=%u "
        //              "BL=%u BR=%u FR=%u BAT=%f",
        //              data[6] ? "ANGLE" : "RATE", data[0], data[1], data[2], data[3], data[4], data[5], data[7],
        //              data[8], data[9], adcPacket[10], adcPacket[11], adcPacket[12], adcPacket[13],
        //              mapf(adcPacket[14], 0, 4096, 0, 16.8));
        // }
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
