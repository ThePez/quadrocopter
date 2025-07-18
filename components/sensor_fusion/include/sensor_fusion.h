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

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#define FUSION_CS_PIN 25
#define FUSION_RESET_PIN 33
#define FUSION_INT_PIN 34

void fusion_init(spi_host_device_t spiHost, void* handler);
void fusion_task(void* param);

#endif
