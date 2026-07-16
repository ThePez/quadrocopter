/*
 *****************************************************************************
 * File: rx_task.c
 * Author: Jack Cairns
 * Date: 16-07-2026
 *****************************************************************************
 */

#include "rx_task.h"

#include "common_functions.h"
#include "espnow_comm.h"
#include "remote.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define TAG "RX_TASK"

#define RX_TASK_STACK (configMINIMAL_STACK_SIZE * 2)
#define RX_TASK_PRIO (tskIDLE_PRIORITY + 2)

// Consumes incoming ESP-NOW config packets
static void rx_task(void* pvParameters) {
    ARG_UNUSED(pvParameters);
    struct wifi_packet_t packet;

    while (1) {
        xQueueReceive(wifiQueue, &packet, portMAX_DELAY);
        switch (packet.packet_id) {
        case REMOTE_CFG:
            remote_config_handle_update(&packet.data.remote_config);
            break;

        case REMOTE_CFG_STORE:
            remote_config_handle_save();
            break;

        default:
            ESP_LOGW(TAG, "Unexpected packet id %d", packet.packet_id);
            break;
        }
    }
}

esp_err_t rx_task_init(void) {
    BaseType_t result = xTaskCreate(rx_task, "RX_TASK", RX_TASK_STACK, NULL, RX_TASK_PRIO, NULL);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
