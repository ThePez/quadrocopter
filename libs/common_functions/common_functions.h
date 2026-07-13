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
#include <esp_log.h>

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

#define CHECK_ERR(code, msg, ...)                                                                  \
    do {                                                                                           \
        esp_err_t _err = (code);                                                                   \
        if (_err != ESP_OK) {                                                                      \
            ESP_LOGE(TAG, msg, ##__VA_ARGS__);                                                     \
            return _err;                                                                            \
        }                                                                                          \
    } while (0)

#define CHECK_ERR_NO_LOG(code)                                                                     \
    do {                                                                                           \
        esp_err_t _err = (code);                                                                   \
        if (_err != ESP_OK) {                                                                      \
            return _err;                                                                            \
        }                                                                                          \
    } while (0)

enum flight_mode_t {
    ACRO,     // ACRO is rate control (FPV mode)
    STABILISE // STABILISE is standard mode (angles)
};

/**
 * @brief Initializes the specified SPI bus (HSPI or VSPI) and its access mutex.
 *
 * Configures the bus's MISO/MOSI/CLK pins for the current target and calls
 * spi_bus_initialize(). Each bus is only initialized once per boot - a
 * repeat call for an already-initialized host returns ESP_ERR_INVALID_ARG.
 *
 * @param host The SPI host to initialize (e.g., HSPI_HOST_COMPAT or VSPI_HOST_COMPAT).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if already initialized, or an error code.
 */
esp_err_t spi_bus_setup(spi_host_device_t host);

/**
 * @brief Initializes the I2C master bus and its access mutex.
 *
 * Uses the target-specific I2C_SCL/I2C_SDA pins with internal pull-ups
 * enabled. The handle is heap-allocated; the caller owns the returned pointer.
 *
 * @return Pointer to a newly allocated I2C master bus handle.
 */
i2c_master_bus_handle_t* i2c_bus_setup(void);

/**
 * @brief Logs a snapshot of all FreeRTOS tasks (name, state, priority, stack, ID) via printf.
 *
 * @return ESP_OK on success, ESP_FAIL if the buffer allocation for vTaskList() failed.
 */
esp_err_t print_task_stats(void);

/**
 * @brief Linearly maps a value from one range to another.
 *
 * @param x       Input value.
 * @param in_min  Lower bound of the input range.
 * @param in_max  Upper bound of the input range.
 * @param out_min Lower bound of the output range.
 * @param out_max Upper bound of the output range.
 * @return x rescaled from [in_min, in_max] into [out_min, out_max].
 */
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief Clamps a value to the inclusive range [min, max].
 *
 * @param value Input value.
 * @param min   Lower bound.
 * @param max   Upper bound.
 * @return value, or min/max if it falls outside the range.
 */
float constrainf(float value, float min, float max);

extern SemaphoreHandle_t spiHMutex;
extern SemaphoreHandle_t spiVMutex;
extern SemaphoreHandle_t i2cMutex;

#endif