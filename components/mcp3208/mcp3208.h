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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define MCPx_CS_PIN_DRONE 26 // Drone
#define MCPx_CS_PIN_REMOTE 25 // Remote

#define MCPx_STACK (configMINIMAL_STACK_SIZE * 2)
#define MCPx_PRIORITY (tskIDLE_PRIORITY + 4)
#define MCPx_QUEUE_LENGTH 5
#define MCPx_DELAY (pdMS_TO_TICKS(50))

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
#define MCP3208_MAX_CHANNELS 8

#define MCP3208_SINGLE_CHANNEL_MASK(channel) (0x0600 | ((channel & 0x0007) << 6))
#define MCP3208_DIFFERENTIAL_CHANNEL_MASK(channel) (0x0400 | ((channel & 0x0007) << 6))

    /**
     * Initializes and starts the FreeRTOS controller task for the MCPx ADC module.
     *
     * Allocates task parameters, sets the selected active ADC channels, and
     * spawns the task responsible for reading values from the MCP3208 device.
     *
     * @param spiMutex Pointer to the SPI mutex used to ensure exclusive bus access.
     * @param channels Bitmask defining which channels should be sampled.
     * @param spiHost SPI host ID where the MCP3208 is connected.
     */
    void
    mcpx_task_init(SemaphoreHandle_t* spiMutex, uint8_t channels, spi_host_device_t spiHost, uint8_t cs);

extern TaskHandle_t mcpxTaskHandle;
extern QueueHandle_t mcpxQueue;

#endif
