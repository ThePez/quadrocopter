/*
 *****************************************************************************
 * File: input_task.c
 * Author: Jack Cairns
 * Date: 05-07-2026
 * Brief: Receives remote packets, maps stick input to commands, sends config acks.
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
#define INPUT_TASK_PRIO  (tskIDLE_PRIORITY + 2)

static TaskHandle_t inputTaskHandle = NULL;
static TaskHandle_t ackTaskHandle = NULL;

static QueueHandle_t ackQueue = NULL;

// Handles a remote data update
static void handle_remote_update(struct remote_telemetry_t *remote)
{
    xSemaphoreTake(cfgMutex, portMAX_DELAY);
    float minThrottle = (float)droneCfg.min_throttle;
    float maxThrottle = (float)droneCfg.max_throttle;
    float maxRate = (float)droneCfg.max_rate;
    xSemaphoreGive(cfgMutex);

    float reading = mapf(remote->throttle, ADC_MIN, ADC_MAX, minThrottle, maxThrottle);
    remoteIn.throttle = (reading < minThrottle + 20) ? minThrottle : reading;
    // Pitch Input
    reading = mapf(remote->pitch, ADC_MIN, ADC_MAX, -maxRate, maxRate);
    remoteIn.pitch = (fabsf(reading) < 5) ? 0 : reading;
    // Roll Input
    reading = mapf(remote->roll, ADC_MIN, ADC_MAX, -maxRate, maxRate);
    remoteIn.roll = (fabsf(reading) < 5) ? 0 : reading;
    // Yaw Input (inverted due to physical position of joystick on remote)
    reading = -mapf(remote->yaw, ADC_MIN, ADC_MAX, -maxRate, maxRate);
    remoteIn.yaw = (fabsf(reading) < 5) ? 0 : reading;

    // Arm only with the throttle stick down, and never while the angle failsafe is
    // active. This also applies after a lost link: when packets resume the drone
    // stays disarmed until the stick is lowered, so it can't spin up on its own.
    uint8_t justArmed = 0;
    xSemaphoreTake(droneData.mutex, portMAX_DELAY);
    droneData.lastRemoteTime = esp_timer_get_time();
    if (!droneData.armed && !(droneData.status_mask & ANGLE_FAIL) &&
        remoteIn.throttle < minThrottle + IDLE_OFFSET) {
        droneData.armed = 1;
        justArmed = 1;
    }

    xSemaphoreGive(droneData.mutex);
    if (justArmed) {
        ESP_LOGI(TAG, "Armed");
    }

    // Flight mode
    set_flight_mode((remote->flight_mode == ACRO) ? ACRO : STABILISE);
}

static void send_bridge_ack(enum esp_now_packet_id type, uint16_t status)
{
    struct wifi_packet_t packet = {.packet_id = type};
    packet.data.result.status = status;
    xQueueSendToBack(ackQueue, &packet, 0);
}

static void ack_task(void *pvParams)
{
    ARG_UNUSED(pvParams);

    struct wifi_packet_t packet = {0};

    while (1) {
        xQueueReceive(ackQueue, &packet, portMAX_DELAY);
        xSemaphoreTake(wifiSendSemaphore, portMAX_DELAY);
        esp_err_t result = esp_now_send_packet(bridge_mac, &packet);
        if (result != ESP_OK) {
            xSemaphoreGive(wifiSendSemaphore);
        }
    }
}

// Main input task loop: blocks on wifiQueue and dispatches REMOTE, PID_CONFIG,
// DRONE_CONFIG, and CONFIG_SAVE packets to their respective handlers.
static void input_control(void *pvParams)
{
    ARG_UNUSED(pvParams);

    struct wifi_packet_t packet = {0};
    esp_err_t err = ESP_OK;

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
            err = pid_handle_config_update(&packet.data.pid_config);
            send_bridge_ack(PID_ACK, err);
            break;

        case DRONE_CFG:
            ESP_LOGI(TAG, "Drone config update");
            err = drone_config_handle_update(&packet.data.drone_config);
            send_bridge_ack(DRONE_CFG_ACK, err);
            break;

        case DRONE_CFG_STORE:
            err = drone_config_handle_save();
            send_bridge_ack(DRONE_CFG_ACK, err);
            break;

        default:
            ESP_LOGW(TAG, "Unexpected packet id %d", packet.packet_id);
            break;
        }
    }
}

esp_err_t input_task_init(void)
{
    ackQueue = xQueueCreate(10, sizeof(struct wifi_packet_t));
    BaseType_t result = xTaskCreatePinnedToCore(input_control, "INPUT_TASK", INPUT_TASK_STACK, NULL,
                                                INPUT_TASK_PRIO, &inputTaskHandle, (BaseType_t)0);
    BaseType_t res2 = xTaskCreatePinnedToCore(ack_task, "ACK_TASK", INPUT_TASK_STACK, NULL,
                                              INPUT_TASK_PRIO, &ackTaskHandle, (BaseType_t)0);
    return (result == pdPASS && res2 == pdPASS && ackQueue) ? ESP_OK : ESP_FAIL;
}
