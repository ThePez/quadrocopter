/*
 *****************************************************************************
 * File: sensor-fusion.h
 * Author: Jack Cairns
 * Date: 17-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <math.h>
#include <string.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define FUSION_CS_PIN 25
#define FUSION_RESET_PIN 33
#define FUSION_INT_PIN 34

#define FUSION_QUEUE_LENGTH 5

typedef struct {
    spi_host_device_t spiHost;
    SemaphoreHandle_t* spiMutex;
} FusionParams_t;

typedef struct {
    float qw;
    float qx;
    float qy;
    float qz;
    float gx;
    float gy;
    float gz;
} SensorData_t;

void fusion_spi_init(spi_host_device_t spiBus);
void fusion_interrupt_init(void* handler);
void fusion_reset_init(void);
void quat_to_euler(float qw, float qx, float qy, float qz, float* roll, float* pitch, float* yaw);
void fusion_init(spi_host_device_t spiHost, void* handler);
void fusion_task(void* pvParam);

extern TaskHandle_t fusionTaskHandle;
extern QueueHandle_t fusionQueue;

#endif
