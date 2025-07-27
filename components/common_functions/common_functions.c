/*
 *****************************************************************************
 * File: common_inits.c
 * Author: Jack Cairns
 * Date: 23-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "common_functions.h"

static uint8_t spiVBusInitialised = 0;
static uint8_t spiHBusInitialised = 0;

SemaphoreHandle_t spiHMutex = NULL;
SemaphoreHandle_t spiVMutex = NULL;
SemaphoreHandle_t i2cMutex = NULL;

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
void spi_bus_setup(spi_host_device_t host) {

    if ((host == VSPI_HOST && spiVBusInitialised) || (host == HSPI_HOST && spiHBusInitialised)) {
        return;
    }

    spi_bus_config_t busConfig = {
        .miso_io_num = (host == HSPI_HOST) ? HSPI_MISO : VSPI_MISO,
        .mosi_io_num = (host == HSPI_HOST) ? HSPI_MOSI : VSPI_MOSI,
        .sclk_io_num = (host == HSPI_HOST) ? HSPI_CLK : VSPI_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(host, &busConfig, SPI_DMA_CH_AUTO));

    if (host == HSPI_HOST) {
        spiHBusInitialised = 1;
        spiHMutex = xSemaphoreCreateMutex();
    } else if (host == VSPI_HOST) {
        spiVBusInitialised = 1;
        spiVMutex = xSemaphoreCreateMutex();
    }
}

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
i2c_master_bus_handle_t* i2c_bus_setup(void) {

    i2c_master_bus_config_t i2cMasterConfig = {
        .clk_source = I2C_CLK_SRC_DEFAULT,    // Default
        .i2c_port = -1,                       // Auto select
        .scl_io_num = GPIO_NUM_22,            // Default
        .sda_io_num = GPIO_NUM_21,            // Default
        .glitch_ignore_cnt = 7,               // Default
        .flags.enable_internal_pullup = true, // Default
    };

    i2c_master_bus_handle_t busHandle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2cMasterConfig, &busHandle));
    i2c_master_bus_handle_t* handle = pvPortMalloc(sizeof(i2c_master_bus_handle_t));
    *handle = busHandle;
    i2cMutex = xSemaphoreCreateMutex();
    return handle;
}

/**
 * @brief Prints a table of current FreeRTOS task statistics.
 *
 * Outputs task name, state, priority, stack high-water mark, and task ID
 * to the standard output using `vTaskList()`. Useful for debugging and
 * monitoring task resource usage.
 *
 * @note Dynamically allocates memory for the task list buffer.
 */
void print_task_stats(void) {

    uint16_t numTasks = uxTaskGetNumberOfTasks();
    uint16_t bufferLength = numTasks * 50;
    char* taskListBuffer = pvPortMalloc(bufferLength * sizeof(char));
    if (taskListBuffer == NULL) {
        return; // Malloc Failed
    }

    vTaskList(taskListBuffer);
    printf("\r\nTask          State  Priority   Stack\tID\r\n");
    printf("=============================================\r\n");
    printf("%s\r\n", taskListBuffer);
    // Free memory
    vPortFree(taskListBuffer);
}

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
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {

    float value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return value;
}

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
float constrainf(float value, float min, float max) {
    if (value > max) {
        value = max;
    }

    if (value < min) {
        value = min;
    }

    return value;
}
