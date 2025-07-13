/*
 *****************************************************************************
 * File: common.c
 * Author: Jack Cairns
 * Date: 13-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "common.h"

static uint8_t spiVBusInitialised = 0;
static uint8_t spiHBusInitialised = 0;

SemaphoreHandle_t spiHMutex = NULL;
SemaphoreHandle_t spiVMutex = NULL;
SemaphoreHandle_t i2cMutex = NULL;

/* spi_bus_setup()
 * ----------------
 * Configures and initializes the specified SPI bus (HSPI or VSPI).
 *
 * Each bus is initialized only once per boot. Pins are configured
 * depending on which host is requested.
 *
 * Parameters:
 *   host - The SPI bus to initialize (HSPI_HOST or VSPI_HOST).
 *
 * Dependencies:
 *   - Uses ESP-IDF SPI Master driver.
 *   - Updates global flags spiHBusInitialised and spiVBusInitialised.
 */
void spi_bus_setup(spi_host_device_t host) {

    if ((host == VSPI_HOST && spiVBusInitialised) || (host == HSPI_HOST && spiHBusInitialised)) {
        return;
    }

    spi_bus_config_t bus_config = {
        .miso_io_num = (host == HSPI_HOST) ? HSPI_MISO : VSPI_MISO,
        .mosi_io_num = (host == HSPI_HOST) ? HSPI_MOSI : VSPI_MOSI,
        .sclk_io_num = (host == HSPI_HOST) ? HSPI_CLK : VSPI_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO));

    if (host == HSPI_HOST) {
        spiHBusInitialised = 1;
        spiHMutex = xSemaphoreCreateMutex();
    } else if (host == VSPI_HOST) {
        spiVBusInitialised = 1;
        spiVMutex = xSemaphoreCreateMutex();
    }
}

/* i2c_bus_setup()
 * ---------------
 *
 */
i2c_master_bus_handle_t* i2c_bus_setup(void) {

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,    // Default
        .i2c_port = -1,                       // Auto select
        .scl_io_num = GPIO_NUM_22,            // Default
        .sda_io_num = GPIO_NUM_21,            // Default
        .glitch_ignore_cnt = 7,               // Default
        .flags.enable_internal_pullup = true, // Default
    };

    i2c_master_bus_handle_t busHandle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &busHandle));
    i2c_master_bus_handle_t* handle = malloc(sizeof(i2c_master_bus_handle_t));
    *handle = busHandle;
    i2cMutex = xSemaphoreCreateMutex();
    return handle;
}

/* print_task_stats()
 * ------------------
 * Prints a formatted list of all currently running FreeRTOS tasks,
 * including their state, priority, stack usage, and task ID.
 *
 * Helpful for debugging stack usage and system health.
 *
 * Uses vTaskList() from FreeRTOS and outputs to standard printf().
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
