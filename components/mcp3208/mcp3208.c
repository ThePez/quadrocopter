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

#include <math.h>
#include <stdio.h>

// SPI device handle
spi_device_handle_t mcp3208SpiHandle = NULL;

/**
 * @brief Initializes the SPI interface for the MCP3208 ADC.
 *
 * Sets the SPI configuration including clock speed, chip select pin,
 * SPI mode (CPOL = 0, CPHA = 0), and queue size. Registers the device
 * on the specified SPI bus.
 *
 * @param spiBus SPI bus to which the MCP3208 is connected (e.g., HSPI_HOST).
 */
void mcp3208_spi_init(spi_host_device_t spiBus) {

    // Setup the SPI for the radio module
    spi_device_interface_config_t deviceConfig = {
        .clock_speed_hz = 1000000,      // 1 MHz Clock speed
        .spics_io_num = MCP3208_CS_PIN, // Chip Select pin for device
        .queue_size = 1,                // Number of pending transactions allowed
        .mode = 0                       // SPI mode, representing a pair of (CPOL, CPHA). CPOL = 1 CPHA = 1
    };

    ESP_ERROR_CHECK(spi_bus_add_device(spiBus, &deviceConfig, &mcp3208SpiHandle));
}

/**
 * @brief Performs MCP3208 startup procedure and initializes SPI communication.
 *
 * Cycles the chip select (CS) pin to reset the MCP3208 and calls
 * the SPI initialization function to prepare for communication.
 *
 * @param spiBus SPI bus to which the MCP3208 is connected.
 */
void mcp3208_init(spi_host_device_t spiBus) {

    gpio_set_direction(MCP3208_CS_PIN, GPIO_MODE_OUTPUT);
    // Cycle the CS pin upon startup
    gpio_set_level(MCP3208_CS_PIN, 0);
    esp_rom_delay_us(50);
    gpio_set_level(MCP3208_CS_PIN, 1);
    esp_rom_delay_us(50);
    gpio_set_level(MCP3208_CS_PIN, 0);

    mcp3208_spi_init(spiBus);
}

/**
 * @brief Reads an analog value from the specified MCP3208 channel.
 *
 * Constructs and sends a read command to the MCP3208 over SPI, then
 * receives and parses the 12-bit ADC result.
 *
 * @param channel ADC channel to read (0–7).
 * @param type Conversion type: 1 for single-ended, 0 for differential.
 *
 * @return 12-bit ADC result from the specified channel.
 */
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

    ESP_ERROR_CHECK(spi_device_transmit(mcp3208SpiHandle, &transaction));

    // Format the output to only include the final 12 bits.
    uint16_t output = rxBuffer[2] | ((rxBuffer[1] & 0x0F) << 8);
    return output;
}
