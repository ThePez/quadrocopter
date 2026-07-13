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

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

struct button_state_t {
    uint64_t prevFalling;
    uint64_t prevRising;
    gpio_num_t pin;
    uint8_t pushAllowed;
    uint32_t notifyBit;
};

struct remote_state_t {
    enum flight_mode_t flightMode;
    uint8_t armed;
    uint8_t emergency;
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

#define MODE_BUTTON_BIT (1UL << 0)
#define EMERGENCY_BUTTON_BIT (1UL << 1)
#define TIMER_BIT (1UL << 31)

////////////////////////////// Global Variables //////////////////////////////

TaskHandle_t remoteTaskHandle = NULL;

// Button states
struct button_state_t mode_button = {
    .pin = MODE_BUTTON_PIN, .pushAllowed = 1, .notifyBit = MODE_BUTTON_BIT};
struct button_state_t emergency_button = {
    .pin = SHUTOFF_BUTTON_PIN, .pushAllowed = 1, .notifyBit = EMERGENCY_BUTTON_BIT};

//////////////////////////////////////////////////////////////////////////////

// Debounced GPIO ISR for a button pin: on a valid falling edge, notifies
// remote_controller() via the button's notify bit.
static void IRAM_ATTR intr_handler(void* args) {

    struct button_state_t* button = (struct button_state_t*) args;
    int state = gpio_get_level(button->pin);
    uint64_t currentTick = esp_timer_get_time();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (!state && button->pushAllowed && (currentTick - button->prevRising >= DEBOUNCE_US)) {
        // Falling edge
        button->prevFalling = currentTick;
        button->pushAllowed = 0;
        xTaskNotifyFromISR(remoteTaskHandle, button->notifyBit, eSetBits,
                           &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else if (state && (currentTick - button->prevFalling >= DEBOUNCE_US)) {
        // Rising edge
        button->prevRising = currentTick;
        button->pushAllowed = 1;
    }
}

// Configures a button's GPIO as an interrupt input and installs the given ISR handler.
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

void IRAM_ATTR remote_callback(void* args) {
    ARG_UNUSED(args);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (remoteTaskHandle) {
        xTaskNotifyFromISR(remoteTaskHandle, TIMER_BIT, eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

esp_err_t remote_timer_init(void) {
    const esp_timer_create_args_t args = {.callback = remote_callback,
                                          .dispatch_method = ESP_TIMER_ISR};
    esp_timer_handle_t remote_timer = NULL;
    CHECK_ERR(esp_timer_create(&args, &remote_timer), "Timer create failed");
    CHECK_ERR(esp_timer_start_periodic(remote_timer, 50000), "Timer start failed");
    ESP_LOGI(TAG, "Remote Timer Initialised");
    return ESP_OK;
}

// One-time hardware bring-up: ESP-NOW paired to the drone, button
// interrupts, the SPI bus and MCP3208 ADC task, and the polling timer.
static esp_err_t hardware_init(void) {
    uint8_t* macs[] = {drone_mac};
    CHECK_ERR_NO_LOG(esp_now_module_init(macs, 1));
    CHECK_ERR_NO_LOG(interrupt_init(intr_handler, &mode_button));
    CHECK_ERR_NO_LOG(interrupt_init(intr_handler, &emergency_button));
    CHECK_ERR(spi_bus_setup(VSPI_HOST), "SPI bus failed init");
    // Remote MCP3208 cs pin is 25
    CHECK_ERR(mcpx_task_init(&spiVMutex, 0x1F, VSPI_HOST, MCPx_CS_PIN_REMOTE),
              "MCPx Task creation failed");

    CHECK_ERR_NO_LOG(remote_timer_init());

    return ESP_OK;
}

// Toggles flight mode (and the status LED) on a mode-button press, if armed.
static void handle_mode_button(struct remote_state_t* state) {
    if (!state->armed) {
        return;
    }

    state->flightMode = (state->flightMode != ACRO) ? ACRO : STABILISE;
    gpio_set_level(STATUS_LED, (state->flightMode == STABILISE) ? 1 : 0);
    ESP_LOGI(TAG, "Flight Mode %s", (state->flightMode == STABILISE) ? "Stabilise" : "Acro");

    // Consume the set MODE bit
    xTaskNotifyWait(MODE_BUTTON_BIT, 0, NULL, 0);
}

// Sets the emergency flag on an emergency-button press, if armed.
static void handle_emergency_button(struct remote_state_t* state) {
    if (!state->armed) {
        return;
    }

    state->emergency = 1;
    ESP_LOGE(TAG, "Emergancy");

    // Consume the set MODE & EMERGENCY bits
    xTaskNotifyWait(MODE_BUTTON_BIT | EMERGENCY_BUTTON_BIT, 0, NULL, 0);
}

// Periodic tick: triggers an ADC read, handles arm/re-arm logic from the
// joystick-press gesture, and sends the resulting REMOTE packet to the drone.
static void handle_timer_tick(struct remote_state_t* state, uint32_t notifyValue,
                              struct wifi_packet_t* packet) {
    // Notify the MCPx task to perform an ADC reading, then wait for the reply
    xTaskNotify(mcpxTaskHandle, 0, eNoAction);
    uint16_t adcValues[MCP3208_MAX_CHANNELS];
    if (xQueueReceive(mcpxQueue, adcValues, REMOTE_DELAY) != pdTRUE) {
        return;
    }

    // Check if now armed
    if (!state->armed && (notifyValue & MODE_BUTTON_BIT) && (notifyValue & EMERGENCY_BUTTON_BIT)) {
        if (!state->emergency) {
            state->armed = 1;
            ESP_LOGW(TAG, "REMOTE ARMED - communication is now enabled.");
        } else if (adcValues[2] < 50) {
            state->emergency = 0;
            state->armed = 1;
            ESP_LOGW(TAG, "REMOTE REARMED - communication is now enabled.");
        }

        if (state->armed) {
            // Consume the ARMING bits now that arming has happened
            xTaskNotifyWait(MODE_BUTTON_BIT | EMERGENCY_BUTTON_BIT, 0, NULL, 0);
        }
    }

    struct remote_telemetry_t* wifiData = &packet->data.remote;
    wifiData->flight_mode = state->flightMode;
    wifiData->throttle = (state->emergency) ? 0 : adcValues[2];
    wifiData->pitch = adcValues[1];
    wifiData->roll = adcValues[0];
    wifiData->yaw = adcValues[3];

    // Send the resulting packet to the drone
    if (state->armed) {
        esp_now_send_packet(drone_mac, packet);
    }

    if (state->emergency) {
        state->armed = 0;
    }
}

// Main remote task: performs hardware init, then loops waiting on button and
// timer notifications and dispatching them to their handlers.
static void remote_controller(void* pvParams) {
    (void) pvParams;
    if (hardware_init()) {
        // This is unrecoverable -> restart the device
        esp_restart(); // This function doesn't return
    }

    struct wifi_packet_t packet = {.packet_id = REMOTE};
    // Idle until all queue's are created
    while (!mcpxQueue || !wifiQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    struct remote_state_t state = {.flightMode = ACRO};
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
        xTaskNotifyWait(0, TIMER_BIT, &notifyValue, portMAX_DELAY);
        if (notifyValue & MODE_BUTTON_BIT) {
            handle_mode_button(&state);
        }

        if (notifyValue & EMERGENCY_BUTTON_BIT) {
            handle_emergency_button(&state);
        }

        if (notifyValue & TIMER_BIT) {
            handle_timer_tick(&state, notifyValue, &packet);
        }
    }
}

esp_err_t init_remote(void) {
    BaseType_t result = xTaskCreate(remote_controller, "REMOTE_TASK", REMOTE_STACK, NULL,
                                    REMOTE_PRIO, &remoteTaskHandle);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
