/*
 *****************************************************************************
 * File: common.h
 * Author: Jack Cairns
 * Date: 23-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef COMMON_H
#define COMMON_H

// STD C lib headers
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// KConfig header
#include "sdkconfig.h"

// ESP-IDF Prebuilts
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// SPI Pins
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_CLK 14
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_CLK 18

// I2C Pins
#define I2C_SCL 22
#define I2C_SDA 21

void spi_bus_setup(spi_host_device_t host);
i2c_master_bus_handle_t* i2c_bus_setup(void);
void print_task_stats(void);

extern SemaphoreHandle_t spiHMutex;
extern SemaphoreHandle_t spiVMutex;
extern SemaphoreHandle_t i2cMutex;

#endif