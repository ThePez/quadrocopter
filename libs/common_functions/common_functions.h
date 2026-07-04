/*
 *****************************************************************************
 * File: common_functions.h
 * Author: Jack Cairns
 * Brief: ESP32 and ESP32-S3 compatible common functions
 *****************************************************************************
 */

#ifndef COMMON_H
#define COMMON_H

#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <esp_err.h>

#define ARG_UNUSED(arg) ((void) arg)

// Handle differences between ESP32 and ESP32-S3
#ifdef CONFIG_IDF_TARGET_ESP32

#define HSPI_HOST_COMPAT HSPI_HOST
#define VSPI_HOST_COMPAT VSPI_HOST

// SPI pins
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_CLK 14
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_CLK 18

// I2C Pins
#define I2C_SCL 22
#define I2C_SDA 21

#else
// ESP32-S3, ESP32-C3, etc use SPI2/SPI3 naming
#define HSPI_HOST_COMPAT SPI2_HOST
#define VSPI_HOST_COMPAT SPI3_HOST

// ESP32-S3 SPI pins (commonly used)
#define HSPI_MISO 13
#define HSPI_MOSI 11
#define HSPI_CLK 12
#define VSPI_MISO 37
#define VSPI_MOSI 35
#define VSPI_CLK 36

// ESP32-S3 I2C pins (commonly used)
#define I2C_SCL 9
#define I2C_SDA 8

#endif

#define CHECK_ERR(code, msg)                                                                       \
    do {                                                                                           \
        esp_err_t err = (code);                                                                    \
        if (err != ESP_OK) {                                                                       \
            ESP_LOGE(TAG, msg);                                                                    \
            return err;                                                                            \
        }                                                                                          \
    } while (0)

#define CHECK_ERR_NO_LOG(code)                                                                     \
    do {                                                                                           \
        esp_err_t err = (code);                                                                    \
        if (err != ESP_OK) {                                                                       \
            return err;                                                                            \
        }                                                                                          \
    } while (0)

typedef enum {
    ACRO,
    STABILISE
} FlightMode_t; // ACRO is rate control (FPV mode), STABILISE is standard mode

esp_err_t spi_bus_setup(spi_host_device_t host);

i2c_master_bus_handle_t* i2c_bus_setup(void);
esp_err_t print_task_stats(void);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
float constrainf(float value, float min, float max);

extern SemaphoreHandle_t spiHMutex;
extern SemaphoreHandle_t spiVMutex;
extern SemaphoreHandle_t i2cMutex;

#endif