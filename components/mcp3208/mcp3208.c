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
#include <stdio.h>

static const char* TAG = "MCP32085";

// SPI device handle
spi_device_handle_t mcp3208SpiHandle = NULL;

esp_err_t mcp3208_spi_init(spi_host_device_t spiBus) {

    // Setup the SPI for the radio module
    spi_device_interface_config_t deviceConfig = {
        .clock_speed_hz = 1000000,      // 1 MHz Clock speed
        .spics_io_num = MCP3208_CS_PIN, // Chip Select pin for device
        .queue_size = 1,                // Number of pending transactions allowed
        .mode = 0                       // SPI mode, representing a pair of (CPOL, CPHA). CPOL = 1 CPHA = 1
    };

    esp_err_t err = spi_bus_add_device(spiBus, &deviceConfig, &mcp3208SpiHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI setip failed");
    }

    return err;
}

esp_err_t mcp3208_init(spi_host_device_t spiBus) {

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

uint16_t mcp3208_read_adc_channel(uint8_t channel, uint8_t type) {

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
        .length = 24,           // Transaction length in bits
        .tx_buffer = txBuffer, // Pointer to transmit buffer
        .rx_buffer = rxBuffer, // Pointer to receive buffer
    };

    esp_err_t err = spi_device_transmit(mcp3208SpiHandle, &transaction);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPI read failed");
        return 0;
    }

    // Format the output to only include the final 12 bits.
    uint16_t output = rxBuffer[2] | ((rxBuffer[1] & 0x0F) << 8);
    return output;
}
