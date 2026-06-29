/*
 *****************************************************************************
 * File: common_functions.c
 * Author: Jack Cairns
 * Date: 23-07-2025
 * Brief: ESP32 and ESP32-S3 compatible common functions
 * REFERENCE: None
 *****************************************************************************
 */

#include "common_functions.h"
#include "esp_log.h"

static uint8_t spiVBusInitialised = 0;
static uint8_t spiHBusInitialised = 0;
static const char* TAG = "COMMON";

SemaphoreHandle_t spiHMutex = NULL;
SemaphoreHandle_t spiVMutex = NULL;
SemaphoreHandle_t i2cMutex = NULL;

esp_err_t spi_bus_setup(spi_host_device_t host) {

    if ((host == VSPI_HOST_COMPAT && spiVBusInitialised) || (host == HSPI_HOST_COMPAT && spiHBusInitialised)) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_config_t busConfig = {
        .miso_io_num = (host == HSPI_HOST_COMPAT) ? HSPI_MISO : VSPI_MISO,
        .mosi_io_num = (host == HSPI_HOST_COMPAT) ? HSPI_MOSI : VSPI_MOSI,
        .sclk_io_num = (host == HSPI_HOST_COMPAT) ? HSPI_CLK : VSPI_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    esp_err_t err = spi_bus_initialize(host, &busConfig, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI host %d already setup.", host);
        err = ESP_OK;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI host setup failed");
        return err;
    }

    if (host == HSPI_HOST_COMPAT) {
        spiHBusInitialised = 1;
        spiHMutex = xSemaphoreCreateMutex();
    } else if (host == VSPI_HOST_COMPAT) {
        spiVBusInitialised = 1;
        spiVMutex = xSemaphoreCreateMutex();
    }

    return err;
}

i2c_master_bus_handle_t* i2c_bus_setup(void) {

    i2c_master_bus_config_t i2cMasterConfig = {
        .clk_source = I2C_CLK_SRC_DEFAULT,    // Default
        .i2c_port = -1,                       // Auto select
        .scl_io_num = I2C_SCL,                // Chip-specific default
        .sda_io_num = I2C_SDA,                // Chip-specific default
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

esp_err_t print_task_stats(void) {

    uint16_t numTasks = uxTaskGetNumberOfTasks();
    uint16_t bufferLength = numTasks * 50;
    char* taskListBuffer = pvPortMalloc(bufferLength * sizeof(char));
    if (taskListBuffer == NULL) {
        return ESP_FAIL; // Malloc Failed
    }

    vTaskList(taskListBuffer);
    printf("\r\nTask          State  Priority   Stack\tID\r\n");
    printf("=============================================\r\n");
    printf("%s\r\n", taskListBuffer);
    // Free memory
    vPortFree(taskListBuffer);
    return ESP_OK;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {

    float value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return value;
}

float constrainf(float value, float min, float max) {
    if (value > max) {
        value = max;
    }

    if (value < min) {
        value = min;
    }

    return value;
}