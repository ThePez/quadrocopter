/*
 *****************************************************************************
 * File: input_task.c
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#include "input_task.h"

#include "common_functions.h"
#include "drone.h"
#include "espnow_comm.h"
#include "pid_task.h"

#include <esp_log.h>
#include <esp_rom_crc.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>

#define TAG "INPUT_TASK"

#define INPUT_TASK_STACK (configMINIMAL_STACK_SIZE * 2)
#define INPUT_TASK_PRIO (tskIDLE_PRIORITY + 2)

static TaskHandle_t inputTaskHandle = NULL;

// Maps a raw REMOTE packet's ADC values into remoteIn setpoints (with
// deadzones on the sticks), applies the flight mode, and marks the drone armed.
static void handle_remote_update(struct remote_telemetry_t* remote) {
    xSemaphoreTake(droneData.mutex, portMAX_DELAY);
    droneData.lastRemoteTime = esp_timer_get_time();

    // Only set armed state if not in angle failsafe state
    if (!(droneData.status_mask & ANGLE_FAIL)) {
        droneData.armed = 1;
    }
    xSemaphoreGive(droneData.mutex);

    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    double minThrottle = droneCfg.min_throttle;
    double maxThrottle = droneCfg.max_throttle;
    double maxRate = droneCfg.max_rate;
    xSemaphoreGive(cfgMutex);

    double reading = mapf(remote->throttle, ADC_MIN, ADC_MAX, minThrottle, maxThrottle);
    remoteIn.throttle = (reading < minThrottle + 20) ? minThrottle : reading;
    // Pitch Input
    reading = mapf(remote->pitch, ADC_MIN, ADC_MAX, -maxRate, maxRate);
    remoteIn.pitch = (fabs(reading) < 5) ? 0 : reading;
    // Roll Input
    reading = mapf(remote->roll, ADC_MIN, ADC_MAX, -maxRate, maxRate);
    remoteIn.roll = (fabs(reading) < 5) ? 0 : reading;
    // Yaw Input (inverted due to physical position of joystick on remote)
    reading = -mapf(remote->yaw, ADC_MIN, ADC_MAX, -maxRate, maxRate);
    remoteIn.yaw = (fabs(reading) < 5) ? 0 : reading;
    // Flight mode
    set_flight_mode((remote->flight_mode == ACRO) ? ACRO : STABILISE);

    // ESP_LOGI(TAG, "SETPOINTS: pitch %lf, roll %lf, yaw %lf, throttle %lf", remoteIn.pitch,
    //          remoteIn.roll, remoteIn.yaw, remoteIn.throttle);
}

// Main input task loop: blocks on wifiQueue and dispatches REMOTE and
// PID_CONFIG packets to their respective handlers.
static void input_control(void* pvParams) {
    ARG_UNUSED(pvParams);

    struct wifi_packet_t packet;

    // Wait for the radio queue to be initialised (should already be done by this point)
    while (!wifiQueue) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Input Task Setup");

    while (1) {
        if (xQueueReceive(wifiQueue, &packet, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (packet.packet_id) {
        case REMOTE:
            handle_remote_update(&packet.data.remote);
            break;

        case PID_CONFIG:
            ESP_LOGI(TAG, "PID Coefficient update");
            pid_handle_config_update(&packet.data.pid_config);
            break;

        default:
            ESP_LOGW(TAG, "Unexpected packet id %d", packet.packet_id);
            break;
        }
    }
}

esp_err_t input_task_init(void) {
    BaseType_t result = xTaskCreatePinnedToCore(input_control, "INPUT_TASK", INPUT_TASK_STACK, NULL,
                                                INPUT_TASK_PRIO, &inputTaskHandle, (BaseType_t) 0);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
