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
spi_device_handle_t mcp3208_spi_handle = NULL;

/* mcp3208_spi_init()
 * -----------------------
 * Initializes the SPI device handle for the MCP3208.
 * Configures the clock speed, SPI mode (CPOL=0, CPHA=0), chip select pin,
 * and adds the device to the specified SPI bus.
 *
 * Parameters:
 *   spi_bus - The SPI bus to use (e.g., HSPI_HOST).
 */
void mcp3208_spi_init(spi_host_device_t spi_bus) {

    // Setup the SPI for the radio module
    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 1000000,      // 1 MHz Clock speed
        .spics_io_num = MCP3208_CS_PIN, // Chip Select pin for device
        .queue_size = 1,                // Number of pending transactions allowed
        .mode = 0                       // SPI mode, representing a pair of (CPOL, CPHA). CPOL = 1 CPHA = 1
    };

    ESP_ERROR_CHECK(spi_bus_add_device(spi_bus, &device_config, &mcp3208_spi_handle));
}

/* mcp3208_init()
 * --------------
 * Power cycles the MCP3208 using the SPI CS pin. Then initialises the device
 * for SPI coms by calling the spi setup function.
 *
 * Parameters:
 *   spi_bus - The SPI bus to use (e.g., HSPI_HOST).
 */
void mcp3208_init(spi_host_device_t spi_bus) {

    gpio_set_direction(MCP3208_CS_PIN, GPIO_MODE_OUTPUT);
    // Cycle the CS pin upon startup
    gpio_set_level(MCP3208_CS_PIN, 0);
    esp_rom_delay_us(50);
    gpio_set_level(MCP3208_CS_PIN, 1);
    esp_rom_delay_us(50);
    gpio_set_level(MCP3208_CS_PIN, 0);

    mcp3208_spi_init(spi_bus);
}

/* mcp3208_read_adc_channel()
 * --------------------------
 * Signals the mcp3208 to perform an ADC read of the selected input channel.
 *
 * Parameters:
 *   channel - the input channel to be used for the conversion
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
    uint8_t tx_buffer[3] = {(uint8_t) (channelMask >> 8), (uint8_t) (channelMask & 0xFF), 0x00};
    uint8_t rx_buffer[3];

    spi_transaction_t transaction = {
        .length = 24,           // Transaction length in bits
        .tx_buffer = tx_buffer, // Pointer to transmit buffer
        .rx_buffer = rx_buffer, // Pointer to receive buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(mcp3208_spi_handle, &transaction));

    // Format the output to only include the final 12 bits.
    uint16_t output = rx_buffer[2] | ((rx_buffer[1] & 0x0F) << 8);
    return output;
}
