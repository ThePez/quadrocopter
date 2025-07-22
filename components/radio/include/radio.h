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
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "hamming.h"
#include "nrf24l01plus.h"

#define RADIO_STACK (configMINIMAL_STACK_SIZE * 2)
#define RADIO_PRIO (tskIDLE_PRIORITY + 4)
#define RADIO_QUEUE_LENGTH 5

///////////////////////////// Structures & Enums /////////////////////////////

// Input struct for setting up the radio module
typedef struct {
    spi_host_device_t spiHost;
    SemaphoreHandle_t* spiMutex;
    SemaphoreHandle_t* setupMutex;
} RadioParams_t;

///////////////////////////////// Prototypes /////////////////////////////////

void radio_module_init(SemaphoreHandle_t* spiMutex);
void encode_packet(void* input, void* output);
void decode_packet(void* input, void* output);

// Task Handles
extern TaskHandle_t radioReceiverTask;
extern TaskHandle_t radioTransmitterTask;
extern TaskHandle_t radioControlTask;
// Queue Handles
extern QueueHandle_t radioReceiverQueue;
extern QueueHandle_t radioTransmitterQueue;

#endif
