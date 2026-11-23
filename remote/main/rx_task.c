/*
 *****************************************************************************
 * File: rx_task.c
 * Author: Jack Cairns
 * Date: 16-07-2026
 * Brief: Receives ESP-NOW packets sent to the remote and returns config acks.
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
#define RX_TASK_PRIO  (tskIDLE_PRIORITY + 2)

static TaskHandle_t ackTaskHandle = NULL;

static QueueHandle_t ackQueue = NULL;

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

// Consumes incoming ESP-NOW config packets
static void rx_task(void *pvParameters)
{
    ARG_UNUSED(pvParameters);
    struct wifi_packet_t packet;
    esp_err_t err = ESP_OK;

    while (1) {
        xQueueReceive(wifiQueue, &packet, portMAX_DELAY);
        switch (packet.packet_id) {
        case REMOTE_CFG:
            err = remote_config_handle_update(&packet.data.remote_config);
            send_bridge_ack(REMOTE_CFG_ACK, err);
            break;

        case REMOTE_CFG_STORE:
            err = remote_config_handle_save();
            send_bridge_ack(REMOTE_CFG_ACK, err);
            break;

        default:
            ESP_LOGW(TAG, "Unexpected packet id %d", packet.packet_id);
            break;
        }
    }
}

esp_err_t rx_task_init(void)
{
    ackQueue = xQueueCreate(10, sizeof(struct wifi_packet_t));
    BaseType_t result = xTaskCreate(rx_task, "RX_TASK", RX_TASK_STACK, NULL, RX_TASK_PRIO, NULL);
    BaseType_t res2 = xTaskCreatePinnedToCore(ack_task, "ACK_TASK", RX_TASK_STACK, NULL,
                                              RX_TASK_PRIO, &ackTaskHandle, (BaseType_t)0);
    return (result == pdPASS && res2 == pdPASS && ackQueue) ? ESP_OK : ESP_FAIL;
}
