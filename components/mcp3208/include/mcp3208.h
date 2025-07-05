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

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define MCP3208_CS_PIN 25

#define MCP3208_SPI_START (1 << 4)

// Channel control bits
#define MCP3208_SINGLE (1 << 3)
#define MCP3208_DIFF (0 << 3)
#define MCP3208_CHANNEL_0 0x0
#define MCP3208_CHANNEL_1 0x1
#define MCP3208_CHANNEL_2 0x2
#define MCP3208_CHANNEL_3 0x3
#define MCP3208_CHANNEL_4 0x4
#define MCP3208_CHANNEL_5 0x5
#define MCP3208_CHANNEL_6 0x6
#define MCP3208_CHANNEL_7 0x7

#define MCP3208_REQUEST(type, channel) ((uint16_t) ((MCP3208_SPI_START | type | channel) << 6))


void mcp3208_spi_init(spi_host_device_t spi_bus);
void mcp3208_init(spi_host_device_t spi_bus);
uint16_t mcp3208_read_adc_channel(uint8_t channel, uint8_t type);

#endif