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

#include "common.h"

#include "hamming.h"
#include "nrf24l01plus.h"

#define RADIO_STACK (configMINIMAL_STACK_SIZE * 2)
#define RADIO_PRIO (tskIDLE_PRIORITY + 4)

///////////////////////////// Structures & Enums /////////////////////////////

typedef struct {
    uint16_t throttle;
    double pitch;
    double roll;
    double yaw;
} ControlSetPoint_t;

///////////////////////////////// Prototypes /////////////////////////////////

// Task Function Prototypes
void radio_receiver_task(void);
void radio_transmitter_task(void);
// Packet encoding & decoding
void encode_packet(void* input, void* output);
void decode_packet(void* input, void* output);

extern TaskHandle_t radioReceiverTask;
extern TaskHandle_t radioTransmitterTask;
extern QueueHandle_t radioReceiverQueue;
extern QueueHandle_t radioTransmitterQueue;

#endif
