/*
 ******************************************************************************
 * File: mcp3208.h
 * Author: Jack Cairns
 * Date: 01-07-2025
 * Brief: Driver for the
 * REFERENCE: None
 ******************************************************************************
 */

#ifndef MCP3208_H
#define MCP3208_H

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

#define MCP3208_CS_PIN 25

#define MCP3208_SPI_START (1 << 10)
#define MCP3208_SINGLE 1
#define MCP3208_DIFF 0

// Channel control bits
#define MCP3208_CHANNEL_0 0x0
#define MCP3208_CHANNEL_1 0x1
#define MCP3208_CHANNEL_2 0x2
#define MCP3208_CHANNEL_3 0x3
#define MCP3208_CHANNEL_4 0x4
#define MCP3208_CHANNEL_5 0x5
#define MCP3208_CHANNEL_6 0x6
#define MCP3208_CHANNEL_7 0x7

#define MCP3208_SINGLE_CHANNEL_MASK(channel) (0x0600 | ((channel & 0x0007) << 6))
#define MCP3208_DIFFERENTIAL_CHANNEL_MASK(channel) (0x0400 | ((channel & 0x0007) << 6))

/**
 * @brief Initializes the SPI interface for the MCP3208 ADC.
 *
 * Sets the SPI configuration including clock speed, chip select pin,
 * SPI mode (CPOL = 0, CPHA = 0), and queue size. Registers the device
 * on the specified SPI bus.
 *
 * @param spiBus SPI bus to which the MCP3208 is connected (e.g., HSPI_HOST).
 */
esp_err_t mcp3208_spi_init(spi_host_device_t spi_bus);

/**
 * @brief Performs MCP3208 startup procedure and initializes SPI communication.
 *
 * Cycles the chip select (CS) pin to reset the MCP3208 and calls
 * the SPI initialization function to prepare for communication.
 *
 * @param spiBus SPI bus to which the MCP3208 is connected.
 */
esp_err_t mcp3208_init(spi_host_device_t spi_bus);

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
uint16_t mcp3208_read_adc_channel(uint8_t channel, uint8_t type);

#endif