/*
 ******************************************************************************
 * File: mcp3208.c
 * Author: Jack Cairns
 * Date: 03-07-2025
 * Brief: Driver for the
 * REFERENCE: None
 ******************************************************************************
 */

#include "mcp3208.h"

#include "common_functions.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define MCPx_STACK (configMINIMAL_STACK_SIZE * 2)
#define MCPx_PRIORITY (tskIDLE_PRIORITY + 4)
#define MCPx_QUEUE_LENGTH 5
#define MCPx_DELAY (pdMS_TO_TICKS(10))
#define TAG "MCPxTask"

struct params_t {
    SemaphoreHandle_t* spiMutex;
    uint8_t channels;
    spi_host_device_t host;
};

TaskHandle_t mcpxTaskHandle = NULL;
QueueHandle_t mcpxQueue = NULL;

static uint8_t MCP3208_CS_PIN = 25;
static struct params_t params = {0};
static spi_device_handle_t mcpxSpiHandle = NULL;

/**
 * Configures and registers the MCP3208 device on the specified SPI bus.
 *
 * Sets up SPI timing, chip-select pin, SPI mode (CPOL = 0, CPHA = 0),
 * and queue size before adding the device to the bus.
 *
 * @param spiBus The SPI host device (e.g., HSPI_HOST or VSPI_HOST).
 * @return ESP_OK on success, or an ESP_FAIL reason on error.
 */
static esp_err_t mcp3208_spi_init(spi_host_device_t spiBus) {
    // Setup the SPI for the radio module
    spi_device_interface_config_t deviceConfig = {
        .clock_speed_hz = 1000000,      // 1 MHz Clock speed
        .spics_io_num = MCP3208_CS_PIN, // Chip Select pin for device
        .queue_size = 1,                // Number of pending transactions allowed
        .mode = 0 // SPI mode, representing a pair of (CPOL, CPHA). CPOL = 1 CPHA = 1
    };

    CHECK_ERR(spi_bus_add_device(spiBus, &deviceConfig, &mcpxSpiHandle), "SPI setip failed");
    return ESP_OK;
}

/**
 * Performs the MCP3208 startup sequence and initializes SPI communication.
 *
 * Cycles the CS line in the correct timing pattern to reset the chip,
 * then calls the SPI initialization routine to prepare the device for use.
 *
 * @param spiBus SPI host to initialize the MCP3208 on.
 * @return ESP_OK if initialization succeeds, otherwise an error code.
 */
static esp_err_t mcp3208_init(spi_host_device_t spiBus) {
    CHECK_ERR(gpio_set_direction(MCP3208_CS_PIN, GPIO_MODE_OUTPUT), "SPI Pin CE failed");
    // Cycle the CS pin upon startup, for ~50us
    gpio_set_level(MCP3208_CS_PIN, 0);
    esp_rom_delay_us(50);
    gpio_set_level(MCP3208_CS_PIN, 1);
    esp_rom_delay_us(50);
    gpio_set_level(MCP3208_CS_PIN, 0);

    return mcp3208_spi_init(spiBus);
}

/**
 * Reads a 12-bit ADC value from the specified MCP3208 channel.
 *
 * Constructs and sends a 3-byte SPI command based on the selected
 * channel and conversion type (single-ended or differential), then
 * extracts and formats the 12-bit result from the returned data.
 *
 * @param channel ADC channel number (0–7).
 * @param type Conversion mode: MCP3208_SINGLE (1) or MCP3208_DIFF (0).
 * @return The 12-bit raw ADC result.
 */
static uint16_t mcp3208_read_adc_channel(uint8_t channel, uint8_t type) {

    uint16_t channelMask =
        (type) ? MCP3208_SINGLE_CHANNEL_MASK(channel) : MCP3208_DIFFERENTIAL_CHANNEL_MASK(channel);

    // Store the request as 2 bytes
    uint8_t txBuffer[3] = {(uint8_t) (channelMask >> 8), (uint8_t) (channelMask & 0xFF), 0x00};
    uint8_t rxBuffer[3];

    spi_transaction_t transaction = {
        .length = 24,          // Transaction length in bits
        .tx_buffer = txBuffer, // Pointer to transmit buffer
        .rx_buffer = rxBuffer, // Pointer to receive buffer
    };

    CHECK_ERR(spi_device_transmit(mcpxSpiHandle, &transaction), "SPI read failed");

    // Format the output to only include the final 12 bits.
    uint16_t output = rxBuffer[2] | ((rxBuffer[1] & 0x0F) << 8);
    return output;
}

static void mcpx_task(void* pvParams) {
    (void) pvParams;
    SemaphoreHandle_t spiMutex = *params.spiMutex;
    uint8_t channelMask = params.channels;
    spi_host_device_t host = params.host;
    uint16_t adcValues[MCP3208_MAX_CHANNELS] = {0};

    while (!mcpxQueue) {
        // Loop to ensure the input queue is created
        mcpxQueue = xQueueCreate(MCPx_QUEUE_LENGTH, sizeof(adcValues));
        vTaskDelay(MCPx_DELAY);
    }

    xSemaphoreTake(spiMutex, portMAX_DELAY);
    esp_err_t err = mcp3208_init(host);
    xSemaphoreGive(spiMutex);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MCPx init failed, shutting down...");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Task initialised");
    uint32_t notify = 0;

    while (1) {

        // Wait for signal to perferm a read
        xTaskNotifyWait(0, 0xFFFFFFFF, &notify, portMAX_DELAY);
        uint16_t retry = 0;

        do {
            if (xSemaphoreTake(spiMutex, MCPx_DELAY) == pdTRUE) {
                for (uint8_t i = 0; i < MCP3208_MAX_CHANNELS; i++) {
                    if ((channelMask & (1 << i)) != 0) {
                        adcValues[i] = mcp3208_read_adc_channel(i, MCP3208_SINGLE);
                    }
                }

                // Return spi mutex
                xSemaphoreGive(spiMutex);
                // Send recieved data to the queue for processing
                xQueueSendToBack(mcpxQueue, adcValues, MCPx_DELAY);
                retry = 0;
            } else {
                retry++;
            }
        } while (retry != 0 && retry < 3);
    }
}

esp_err_t mcpx_task_init(SemaphoreHandle_t* spiMutex, uint8_t channels, spi_host_device_t spiHost,
                         uint8_t cs) {
    params.spiMutex = spiMutex;
    params.channels = channels;
    params.host = spiHost;
    MCP3208_CS_PIN = cs;
    BaseType_t result = xTaskCreate((void*) &mcpx_task, "MCPxTask", MCPx_STACK, NULL,
                                    MCPx_PRIORITY, &mcpxTaskHandle);
    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}