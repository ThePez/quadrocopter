/*
 ******************************************************************************
 * File: remote.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 ******************************************************************************
 */

#include "remote.h"

#include "common_functions.h"
#include "espnow_comm.h"
#include "mcp3208.h"

#include <esp_log.h>
#include <esp_now.h>
#include <esp_rom_crc.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

struct button_state_t {
    uint64_t prevFalling;
    uint64_t prevRising;
    gpio_num_t pin;
    uint8_t pushAllowed;
};

#define REMOTE_STACK (configMINIMAL_STACK_SIZE * 2)
#define REMOTE_PRIO (tskIDLE_PRIORITY + 2)
#define REMOTE_DELAY (pdMS_TO_TICKS(50))

#define TAG "REMOTE"

#define STATUS_LED 2
#define DEBOUNCE_US 50000

#define ADC_MIN 0
#define ADC_MAX 4095

#define MIN_THROTTLE 1000.0
#define MAX_THROTTLE 2000.0

////////////////////////////// Global Variables //////////////////////////////

TaskHandle_t remoteTaskHandle = NULL;

// Button states
struct button_state_t mode_button = {.pin = MODE_BUTTON_PIN, .pushAllowed = 1};
struct button_state_t emergency_button = {.pin = SHUTOFF_BUTTON_PIN, .pushAllowed = 1};

//////////////////////////////////////////////////////////////////////////////

static void IRAM_ATTR intr_handler(void* args) {

    struct button_state_t* button = (struct button_state_t*) args;
    int state = gpio_get_level(button->pin);
    uint64_t currentTick = esp_timer_get_time();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (!state && button->pushAllowed && (currentTick - button->prevRising >= DEBOUNCE_US)) {
        // Falling edge
        button->prevFalling = currentTick;
        button->pushAllowed = 0;
        xTaskNotifyFromISR(remoteTaskHandle, button->pin, eSetValueWithOverwrite,
                           &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else if (state && (currentTick - button->prevFalling >= DEBOUNCE_US)) {
        // Rising edge
        button->prevRising = currentTick;
        button->pushAllowed = 1;
    }
}

static esp_err_t interrupt_init(void* handler, struct button_state_t* button) {

    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT,               // Direction of the pin
        .pin_bit_mask = (1ULL << button->pin), // Pin is listed here as a bit mask
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Enable internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable internal pull-down
        .intr_type = GPIO_INTR_ANYEDGE         // Interrupt on both edges, switch is active low
    };

    // Configure the GPIO-pin
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
    return interrupt_init(intr_handler, &emergency_button);
}

static esp_err_t mode_swap_interrupt_init(void) {
    return interrupt_init(intr_handler, &mode_button);
}

static esp_err_t hardware_init(void) {
    uint8_t* macs[] = {drone_mac};
    esp_now_module_init(macs, 1);

    esp_err_t err = mode_swap_interrupt_init();

    err = emergancy_interrupt_init();
    return err;
}

static void remote_controller(void* pvParams) {
    (void) pvParams;
    if (hardware_init()) {
        esp_restart();
    }

    uint16_t adcValues[5] = {0};
    struct wifi_packet_t packet = {.packet_id = REMOTE};
    struct remote_telemetry_t* wifiData = &packet.data.remote;
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
                        ESP_LOGI(TAG, "Flight Mode %s",
                                 (flightMode == STABILISE) ? "Stabilise" : "Acro");
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

            wifiData->flight_mode = flightMode;
            wifiData->throttle = (emergencyShutdown) ? 0 : adcValues[2];
            wifiData->pitch = adcValues[1];
            wifiData->roll = adcValues[0];
            wifiData->yaw = adcValues[3];
            packet.crc16 = esp_rom_crc16_le(0, (uint8_t*) &packet.data, sizeof(union packet_data));

            // Send the resulting packet to the drone
            if (armed) {
                esp_now_send(drone_mac, (uint8_t*) &packet, sizeof(struct wifi_packet_t));
            }

            if (emergencyShutdown) {
                armed = 0;
            }
        }
    }
}

esp_err_t init_remote(void) {
    spi_bus_setup(VSPI_HOST);
    mcpx_task_init(&spiVMutex, 0x1F, VSPI_HOST, MCPx_CS_PIN_REMOTE); // Remote MCP3208 cs pin is 25
    return xTaskCreate(remote_controller, "REMOTE_TASK", REMOTE_STACK, NULL, REMOTE_PRIO,
                       &remoteTaskHandle);
}
