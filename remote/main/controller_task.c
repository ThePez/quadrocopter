/*
 *****************************************************************************
 * File: controller_task.c
 * Author: Jack Cairns
 * Date: 16-07-2026
 *****************************************************************************
 */

#include "controller_task.h"

#include "common_functions.h"
#include "device_config.h"
#include "espnow_comm.h"
#include "mcp3208.h"
#include "remote.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "CONTROLLER_TASK"

#define CONTROLLER_TASK_STACK (configMINIMAL_STACK_SIZE * 2)
#define CONTROLLER_TASK_PRIO (tskIDLE_PRIORITY + 2)
#define CONTROLLER_DELAY (pdMS_TO_TICKS(50))

#define STATUS_LED 2

#define ADC_MIN 0
#define ADC_MAX 4095

struct remote_state_t {
    enum flight_mode_t flightMode;
    uint8_t armed;
    uint8_t emergency;
};

TaskHandle_t remoteTaskHandle = NULL;

// Remaps a raw ADC reading through its per-channel calibration
static uint16_t apply_joystick_cal(uint16_t raw, struct nvs_joystick_cal_t cal) {
    double adcMid = (ADC_MIN + ADC_MAX) / 2.0;
    double corrected = (raw < cal.centre) ? mapf(raw, cal.min, cal.centre, ADC_MIN, adcMid)
                                          : mapf(raw, cal.centre, cal.max, adcMid, ADC_MAX);
    return (uint16_t) constrainf(corrected, ADC_MIN, ADC_MAX);
}

// Periodic tick: triggers an ADC read, handles arm/re-arm logic from the
// joystick-press gesture, and sends the resulting REMOTE packet to the drone.
static void handle_timer_tick(struct remote_state_t* state, uint32_t notifyValue,
                              struct wifi_packet_t* packet) {
    // Notify the MCPx task to perform an ADC reading, then wait for the reply
    xTaskNotify(mcpxTaskHandle, 0, eNoAction);
    uint16_t adcValues[MCP3208_MAX_CHANNELS];
    if (xQueueReceive(mcpxQueue, adcValues, CONTROLLER_DELAY) != pdTRUE) {
        return;
    }

    // Correct each channel's raw reading for its calibrated centre/travel
    // before anything downstream (arming check, packet fill) uses it
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    struct nvs_joystick_cal_t throttleCal = remoteCfg.throttle;
    struct nvs_joystick_cal_t pitchCal = remoteCfg.pitch;
    struct nvs_joystick_cal_t rollCal = remoteCfg.roll;
    struct nvs_joystick_cal_t yawCal = remoteCfg.yaw;
    xSemaphoreGive(cfgMutex);

    adcValues[0] = apply_joystick_cal(adcValues[0], rollCal);
    adcValues[1] = apply_joystick_cal(adcValues[1], pitchCal);
    adcValues[2] = apply_joystick_cal(adcValues[2], throttleCal);
    adcValues[3] = apply_joystick_cal(adcValues[3], yawCal);

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

// Main remote task
static void remote_controller(void* pvParams) {
    (void) pvParams;
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

esp_err_t controller_task_init(void) {
    BaseType_t result = xTaskCreate(remote_controller, "REMOTE_TASK", CONTROLLER_TASK_STACK, NULL,
                                    CONTROLLER_TASK_PRIO, &remoteTaskHandle);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
