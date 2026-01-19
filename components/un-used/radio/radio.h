/*
 *****************************************************************************
 * File: radio.h
 * Author: Jack Cairns
 * Date: 13-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef RADIO_H
#define RADIO_H

// STD C lib headers
#include <stdint.h>

// ESP-IDF Prebuilts
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define RADIO_STACK (configMINIMAL_STACK_SIZE * 2)
#define RADIO_PRIO (tskIDLE_PRIORITY + 4)
#define RADIO_QUEUE_LENGTH 5

#define RADIO_TX_READY (1 << 10)
#define RADIO_RX_READY (1 << 11)

#define RADIO_PAYLOAD_WIDTH 32

// Input struct for setting up the radio module
typedef struct {
    SemaphoreHandle_t* spiMutex;
} RadioParams_t;

void radio_module_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost);

// Task Handles
extern TaskHandle_t radioReceiverTask;
extern TaskHandle_t radioTransmitterTask;
extern TaskHandle_t radioControlTask;
// Queue Handles
extern QueueHandle_t radioReceiverQueue;
extern QueueHandle_t radioTransmitterQueue;

#endif
