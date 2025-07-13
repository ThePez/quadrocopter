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
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "hamming.h"
#include "nrf24l01plus.h"

#define RADIO_STACK (configMINIMAL_STACK_SIZE * 2)
#define RADIO_PRIO (tskIDLE_PRIORITY + 4)

#define RADIO_RECEIVER_QUEUE_LENGTH 2

///////////////////////////// Structures & Enums /////////////////////////////

typedef struct {
    spi_host_device_t spiHost;
    SemaphoreHandle_t* spiMutex;
    SemaphoreHandle_t* setupMutex;
} RadioParams_t;

///////////////////////////////// Prototypes /////////////////////////////////

// Task Function Prototypes
void radio_control_task(void* pvParams);
void radio_receiver_task(void* pvParams);
void radio_transmitter_task(void* pvParams);
// Packet encoding & decoding
SemaphoreHandle_t* radio_setup(RadioParams_t* params);
void encode_packet(void* input, void* output);
void decode_packet(void* input, void* output);

extern TaskHandle_t radioReceiverTask;
extern TaskHandle_t radioTransmitterTask;
extern TaskHandle_t radioControlTask;

extern QueueHandle_t radioReceiverQueue;
extern QueueHandle_t radioTransmitterQueue;

#endif
