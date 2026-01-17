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

#include "esp_log.h"
#include <math.h>

//////////////////////////// Function Prototypes /////////////////////////////

/**
 * FreeRTOS task responsible for continuously sampling ADC inputs from
 * the MCPx (MCP3208) chip.
 *
 * The task acquires the shared SPI mutex, cycles through all active
 * channels, performs ADC conversions, and sends the results over the
 * MCPx queue every configured delay period (50 ms).
 *
 * @param pvParams Pointer to mcpxInputParams_t
 */
static void mcpx_task(void* pvParams);

/**
 * Configures and registers the MCP3208 device on the specified SPI bus.
 *
 * Sets up SPI timing, chip-select pin, SPI mode (CPOL = 0, CPHA = 0),
 * and queue size before adding the device to the bus.
 *
 * @param spiBus The SPI host device (e.g., HSPI_HOST or VSPI_HOST).
 * @return ESP_OK on success, or an ESP_FAIL reason on error.
 */
static esp_err_t mcp3208_spi_init(spi_host_device_t spiBus);

/**
 * Performs the MCP3208 startup sequence and initializes SPI communication.
 *
 * Cycles the CS line in the correct timing pattern to reset the chip,
 * then calls the SPI initialization routine to prepare the device for use.
 *
 * @param spiBus SPI host to initialize the MCP3208 on.
 * @return ESP_OK if initialization succeeds, otherwise an error code.
 */
static esp_err_t mcp3208_init(spi_host_device_t spiBus);

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
static uint16_t mcp3208_read_adc_channel(uint8_t channel, uint8_t type);

//////////////////////////////////////////////////////////////////////////////

#define TAG "MCPxTask"

static uint8_t MCP3208_CS_PIN = 25;

typedef struct {
    SemaphoreHandle_t* spiMutex; // Spi mutex used for thread safe spi comunication
    uint8_t channels;            // A bit mask containing which channels to read from
    spi_host_device_t host;
} mcpxInputParams_t;

TaskHandle_t mcpxTaskHandle = NULL;
QueueHandle_t mcpxQueue = NULL;

// SPI device handle
spi_device_handle_t mcpxSpiHandle = NULL;

void mcpx_task_init(SemaphoreHandle_t* spiMutex, uint8_t channels, spi_host_device_t spiHost, uint8_t cs) {
    mcpxInputParams_t* params = pvPortMalloc(sizeof(mcpxInputParams_t));
    params->spiMutex = spiMutex;
    params->channels = channels;
    params->host = spiHost;
    MCP3208_CS_PIN = cs;
    xTaskCreate((void*) &mcpx_task, "MCPxTask", MCPx_STACK, params, MCPx_PRIORITY, &mcpxTaskHandle);
}

static void mcpx_task(void* pvParams) {

    mcpxInputParams_t* params = (mcpxInputParams_t*) pvParams;
    SemaphoreHandle_t spiMutex = *params->spiMutex;
    uint8_t numChannels = 0;
    uint8_t channelMask = params->channels;
    spi_host_device_t host = params->host;
    vPortFree(pvParams);
    for (uint8_t i = 0; i < MCP3208_MAX_CHANNELS; i++) {
        if ((channelMask & (1 << i)) != 0) {
            numChannels++;
        }
    }

    // Setup an array to hold the read adc values
    uint16_t* adcValues = pvPortMalloc(sizeof(uint16_t) * numChannels);
    if (adcValues == NULL) {
        ESP_LOGE(TAG, "adc malloc failed, shutting down...");
        vTaskDelete(NULL);
    } else {
        // Set new array to 0
        for (int i = 0; i < numChannels; i++) {
            adcValues[i] = 0;
        }
    }

    while (!mcpxQueue) {
        // Loop to ensure the input queue is created
        mcpxQueue = xQueueCreate(MCPx_QUEUE_LENGTH, sizeof(uint16_t) * numChannels);
        vTaskDelay(MCPx_DELAY);
    }

    xSemaphoreTake(spiMutex, portMAX_DELAY);
    esp_err_t err = mcp3208_init(host); // VSPI_HOST for remote;
    xSemaphoreGive(spiMutex);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MCPx init failed, shutting down...");
        vPortFree(adcValues);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Task initialised");

    while (1) {

        if (xSemaphoreTake(spiMutex, MCPx_DELAY) == pdTRUE) {
            uint8_t pos = 0;
            for (uint8_t i = 0; i < MCP3208_MAX_CHANNELS; i++) {
                // Channels 1 to 4 are joysticks, 0 is slider for the remote
                // Channel 1 is the battery for the drone
                if ((channelMask & (1 << i)) != 0) {
                    adcValues[pos] = mcp3208_read_adc_channel(i, MCP3208_SINGLE);
                    // ESP_LOGI(TAG, "ADC CH%d: %d", pos, adcValues[pos]);
                    pos++;
                }
            }

            // Return spi mutex
            xSemaphoreGive(spiMutex);
            // Send recieved data to the queue for processing
            xQueueSendToBack(mcpxQueue, adcValues, MCPx_DELAY);
        }

        vTaskDelay(MCPx_DELAY);
    }
}

static esp_err_t mcp3208_spi_init(spi_host_device_t spiBus) {

    // Setup the SPI for the radio module
    spi_device_interface_config_t deviceConfig = {
        .clock_speed_hz = 1000000,      // 1 MHz Clock speed
        .spics_io_num = MCP3208_CS_PIN, // Chip Select pin for device
        .queue_size = 1,                // Number of pending transactions allowed
        .mode = 0                       // SPI mode, representing a pair of (CPOL, CPHA). CPOL = 1 CPHA = 1
    };

    esp_err_t err = spi_bus_add_device(spiBus, &deviceConfig, &mcpxSpiHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI setip failed");
    }

    return err;
}

static esp_err_t mcp3208_init(spi_host_device_t spiBus) {

    esp_err_t err = gpio_set_direction(MCP3208_CS_PIN, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI Pin CE failed");
        return err;
    }

    // Cycle the CS pin upon startup, for ~50us
    gpio_set_level(MCP3208_CS_PIN, 0);
    esp_rom_delay_us(50);
    gpio_set_level(MCP3208_CS_PIN, 1);
    esp_rom_delay_us(50);
    gpio_set_level(MCP3208_CS_PIN, 0);

    err = mcp3208_spi_init(spiBus);
    return err;
}

static uint16_t mcp3208_read_adc_channel(uint8_t channel, uint8_t type) {

    uint16_t channelMask;
    if (type) {
        // Single
        channelMask = MCP3208_SINGLE_CHANNEL_MASK(channel);
    } else {
        // Differential
        channelMask = MCP3208_DIFFERENTIAL_CHANNEL_MASK(channel);
    }

    // Store the request as 2 bytes
    uint8_t txBuffer[3] = {(uint8_t) (channelMask >> 8), (uint8_t) (channelMask & 0xFF), 0x00};
    uint8_t rxBuffer[3];

    spi_transaction_t transaction = {
        .length = 24,          // Transaction length in bits
        .tx_buffer = txBuffer, // Pointer to transmit buffer
        .rx_buffer = rxBuffer, // Pointer to receive buffer
    };

    esp_err_t err = spi_device_transmit(mcpxSpiHandle, &transaction);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPI read failed");
        return 0;
    }

    // Format the output to only include the final 12 bits.
    uint16_t output = rxBuffer[2] | ((rxBuffer[1] & 0x0F) << 8);
    return output;
}
