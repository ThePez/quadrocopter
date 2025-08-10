/*
 *****************************************************************************
 * File: common_inits.h
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

// ESP-IDF Prebuilts
#include "driver/i2c_master.h"
#include "driver/spi_master.h"

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

/**
 * @brief Initializes the specified SPI bus (HSPI or VSPI).
 *
 * Configures and initializes the SPI bus pins based on the selected host.
 * Ensures that each bus is only initialized once per boot. Also creates a
 * mutex for synchronizing access to the SPI bus.
 *
 * @param host The SPI host to initialize (e.g., HSPI_HOST or VSPI_HOST).
 *
 * @note Relies on the ESP-IDF SPI Master driver.
 */
esp_err_t spi_bus_setup(spi_host_device_t host);

/**
 * @brief Initializes the I2C master bus with default settings.
 *
 * Sets up the I2C master interface using GPIO 21 for SDA and GPIO 22 for SCL.
 * Pull-up resistors are enabled. The function allocates and returns a pointer
 * to the bus handle, and creates a mutex for synchronized access.
 *
 * @return Pointer to a dynamically allocated I2C master bus handle.
 *
 * @note Uses the ESP-IDF I2C Master driver.
 */
i2c_master_bus_handle_t* i2c_bus_setup(void);

/**
 * @brief Prints a table of current FreeRTOS task statistics.
 *
 * Outputs task name, state, priority, stack high-water mark, and task ID
 * to the standard output using `vTaskList()`. Useful for debugging and
 * monitoring task resource usage.
 *
 * @note Dynamically allocates memory for the task list buffer.
 */
esp_err_t print_task_stats(void);

/**
 * @brief Maps a floating-point number from one range to another.
 *
 * This function linearly transforms a value `x` from the input range
 * [in_min, in_max] to the corresponding value in the output range
 * [out_min, out_max].
 *
 * @param x        The input value to map.
 * @param in_min   The lower bound of the input range.
 * @param in_max   The upper bound of the input range.
 * @param out_min  The lower bound of the output range.
 * @param out_max  The upper bound of the output range.
 *
 * @return The mapped floating-point value in the output range.
 */
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief Constrains a floating-point value to lie within a specified range.
 *
 * This function limits the input value to be within the range [min, max].
 * If the value is greater than max, it returns max.
 * If the value is less than min, it returns min.
 * Otherwise, it returns the original value.
 *
 * @param value  The input value to constrain.
 * @param min    The lower bound of the range.
 * @param max    The upper bound of the range.
 *
 * @return The constrained value within [min, max].
 */
float constrainf(float value, float min, float max);

extern SemaphoreHandle_t spiHMutex;
extern SemaphoreHandle_t spiVMutex;
extern SemaphoreHandle_t i2cMutex;

#endif