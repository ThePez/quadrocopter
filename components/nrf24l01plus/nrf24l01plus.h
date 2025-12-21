/*
 ******************************************************************************
 * File: nrf24l01plus.h
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 ******************************************************************************
 */

#ifndef NRF24L01PLUS_H
#define NRF24L01PLUS_H

// STD C lib headers
#include <stdint.h>

// ESP-IDF Prebuilts
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Task Handles
extern TaskHandle_t radioReceiverTask;
extern TaskHandle_t radioTransmitterTask;
extern TaskHandle_t radioControlTask;
// Queue Handles
extern QueueHandle_t radioReceiverQueue;
extern QueueHandle_t radioTransmitterQueue;

// Free-RTOS defines
#define RADIO_STACK (configMINIMAL_STACK_SIZE * 2)
#define RADIO_PRIO (tskIDLE_PRIORITY + 4)
#define RADIO_QUEUE_LENGTH 5
#define RADIO_TX_READY (1 << 10)
#define RADIO_RX_READY (1 << 11)
#define RADIO_PAYLOAD_WIDTH 32

// Actual Pins numbers used on the ESP chip
#define NRF24L01PLUS_CS_PIN 26
#define NRF24L01PLUS_CE_PIN 27
#define NRF24L01PLUS_IQR_PIN 35

#define NRF24L01PLUS_TX_ADR_WIDTH 5    // 5 unsigned chars TX(RX) address width
#define NRF24L01PLUS_TX_PLOAD_WIDTH 32 // 32 unsigned chars TX payload

// SPI(nRF24L01) commands
#define NRF24L01PLUS_READ_REG 0x00           // Define read command to register
#define NRF24L01PLUS_WRITE_REG 0x20          // Define write command to register
#define NRF24L01PLUS_RD_RX_PLOAD 0x61        // Define RX payload register address
#define NRF24L01PLUS_WR_TX_PLOAD 0xA0        // Define TX payload register address
#define NRF24L01PLUS_FLUSH_TX 0xE1           // Define flush TX register command
#define NRF24L01PLUS_FLUSH_RX 0xE2           // Define flush RX register command
#define NRF24L01PLUS_ACTIVATE 0x50           // ACTIVATE additional features
#define NRF24L01PLUS_REUSE_TX_PL 0xE3        // Define reuse TX payload register command
#define NRF24L01PLUS_R_RX_PL_WID 0x60        // Define Read RX-payload width command
#define NRF24L01PLUS_W_ACK_PAYLOAD 0xA8      // Write payload to be used in ACK packet on pipe PPP
#define NRF24L01PLUS_W_TX_PAYLOAD_NOACK 0xB0 // Used in TX mode, Disable AUTOACK on this specific packet
#define NRF24L01PLUS_OP_NOP 0xFF             // Define No Operation, might be used to read status register

// SPI(nRF24L01) registers(addresses)
#define NRF24L01PLUS_CONFIG 0x00      // 'Config' register address
#define NRF24L01PLUS_EN_AA 0x01       // 'Enable Auto Acknowledgment' register address
#define NRF24L01PLUS_EN_RXADDR 0x02   // 'Enabled RX addresses' register address
#define NRF24L01PLUS_SETUP_AW 0x03    // 'Setup address width' register address
#define NRF24L01PLUS_SETUP_RETR 0x04  // 'Setup Auto. Retrans' register address
#define NRF24L01PLUS_RF_CH 0x05       // 'RF channel' register address
#define NRF24L01PLUS_RF_SETUP 0x06    // 'RF setup' register address
#define NRF24L01PLUS_STATUS 0x07      // 'Status' register address
#define NRF24L01PLUS_OBSERVE_TX 0x08  // 'Observe TX' register address
#define NRF24L01PLUS_RPD 0x09         // 'Carrier Detect' register address
#define NRF24L01PLUS_RX_ADDR_P0 0x0A  // 'RX address pipe0' register address
#define NRF24L01PLUS_RX_ADDR_P1 0x0B  // 'RX address pipe1' register address
#define NRF24L01PLUS_RX_ADDR_P2 0x0C  // 'RX address pipe2' register address
#define NRF24L01PLUS_RX_ADDR_P3 0x0D  // 'RX address pipe3' register address
#define NRF24L01PLUS_RX_ADDR_P4 0x0E  // 'RX address pipe4' register address
#define NRF24L01PLUS_RX_ADDR_P5 0x0F  // 'RX address pipe5' register address
#define NRF24L01PLUS_TX_ADDR 0x10     // 'TX address' register address
#define NRF24L01PLUS_RX_PW_P0 0x11    // 'RX payload width, pipe0' register address
#define NRF24L01PLUS_RX_PW_P1 0x12    // 'RX payload width, pipe1' register address
#define NRF24L01PLUS_RX_PW_P2 0x13    // 'RX payload width, pipe2' register address
#define NRF24L01PLUS_RX_PW_P3 0x14    // 'RX payload width, pipe3' register address
#define NRF24L01PLUS_RX_PW_P4 0x15    // 'RX payload width, pipe4' register address
#define NRF24L01PLUS_RX_PW_P5 0x16    // 'RX payload width, pipe5' register address
#define NRF24L01PLUS_FIFO_STATUS 0x17 // 'FIFO Status Register' register address
#define NRF24L01PLUS_DYNPD 0x1C       // 'Enable dynamic payload length' register address
#define NRF24L01PLUS_FEATURE 0x1D     // Additional features register, needed to enable the additional commands

// SPI(nRF24L01) registers(bitmasks)
#define NRF24L01PLUS_ERX_P0 0x01 // Enable Pipe 0 (register EN_RXADDR)
#define NRF24L01PLUS_ERX_P1 0x02 // Enable Pipe 1 (register EN_RXADDR)
#define NRF24L01PLUS_ERX_P2 0x04 // Enable Pipe 2 (register EN_RXADDR)
#define NRF24L01PLUS_ERX_P3 0x08 // Enable Pipe 3 (register EN_RXADDR)
#define NRF24L01PLUS_ERX_P4 0x10 // Enable Pipe 4 (register EN_RXADDR)
#define NRF24L01PLUS_ERX_P5 0x20 // Enable Pipe 5 (register EN_RXADDR)

#define NRF24L01PLUS_FIFO_RX_EMPTY 0x01
#define NRF24L01PLUS_FIFO_RX_FULL 0x02
#define NRF24L01PLUS_FIFO_TX_EMPTY 0x10
#define NRF24L01PLUS_FIFO_TX_FULL 0x20
#define NRF24L01PLUS_FIFO_TX_REUSE 0x40

#define NRF24L01PLUS_RX_DR 0x40
#define NRF24L01PLUS_TX_DS 0x20
#define NRF24L01PLUS_MAX_RT 0x10

// Set/Reset CE pin
#define NRF_CE_HIGH() gpio_set_level(NRF24L01PLUS_CE_PIN, 1)
#define NRF_CE_LOW() gpio_set_level(NRF24L01PLUS_CE_PIN, 0)

/**
 * Initializes the NRF24L01+ radio driver and starts all radio-related FreeRTOS tasks.
 *
 * This function performs the full radio bring-up sequence, including:
 * - Initializing the NRF24L01+ hardware over SPI
 * - Ensuring the radio event group is created
 * - Creating the receiver and transmitter queues
 * - Launching the control, receiver, and transmitter tasks
 *
 * The SPI mutex must be held during initial hardware configuration to avoid
 * bus contention with other peripherals.
 *
 * @param spiMutex Pointer to the SPI bus mutex for synchronized SPI access.
 * @param spiHost  The SPI host device connected to the NRF24L01+ module.
 */
void radio_module_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost);

/**
 * Initializes the SPI interface for the NRF24L01+ module.
 *
 * Sets clock speed, SPI mode (CPOL=0, CPHA=0), chip select pin,
 * and registers the device on the SPI bus.
 *
 * @param spi_bus SPI host bus to attach the device to.
 * @return ESP_OK on success, ESP_FAIL on configuration error.
 */
esp_err_t nrf24l01plus_spi_init(spi_host_device_t spi_bus);

/**
 * Configures the IRQ pin for falling-edge interrupts and installs the ISR.
 *
 * @param handler ISR function to attach to the IRQ line.
 * @return ESP_OK on success,
 *         ESP_ERR_INVALID_STATE if ISR service already installed,
 *         or other ESP error codes on failure.
 */
esp_err_t nrf24l01plus_interrupt_init(void* handler);

/**
 * Fully initializes the NRF24L01+ transceiver.
 *
 * Sets CE pin direction, installs IRQ handler if provided, initializes SPI,
 * powers up the radio, writes TX/RX addresses, configures channel, data rate,
 * payload size, RF power, and enters RX mode.
 *
 * @param spiBus SPI host device.
 * @param handler Optional IRQ handler (NULL if unused).
 * @return ESP_OK on success, ESP_FAIL on initialization failure.
 */
esp_err_t nrf24l01plus_init(spi_host_device_t spi_bus, void* handler);

/**
 * Sends an 8-bit instruction command to the NRF24L01+ by SPI.
 *
 * @param command Command byte to transmit.
 * @return ESP_OK on success, ESP_FAIL on SPI transmission error.
 */
esp_err_t nrf24l01plus_send_command(uint8_t command);

/**
 * Writes a single byte to an NRF24L01+ register.
 *
 * @param reg_addr Register address.
 * @param val Value to write.
 * @return ESP_OK on success, ESP_FAIL on SPI error.
 */
esp_err_t nrf24l01plus_write_register(uint8_t reg_addr, uint8_t val);

/**
 * Reads a single byte from an NRF24L01+ register.
 *
 * @param reg_addr Register address to read.
 * @return The register value, or 0 on SPI error.
 */
uint8_t nrf24l01plus_read_register(uint8_t reg_addr);

/**
 * Writes multiple bytes to an NRF24L01+ register.
 *
 * Commonly used for writing TX/RX addresses or payload data.
 *
 * @param reg_addr Register address to write to.
 * @param buffer Pointer to the data buffer.
 * @param buffer_len Number of bytes to write.
 * @return ESP_OK on success, ESP_FAIL on SPI error.
 */
esp_err_t nrf24l01plus_write_buffer(uint8_t reg_addr, uint8_t* buffer, int buffer_len);

/**
 * Reads multiple bytes from an NRF24L01+ register and copies them into a buffer.
 *
 * The first received byte is the STATUS register and is skipped.
 *
 * @param regAddress Register address to read from.
 * @param buffer Destination buffer to store received bytes.
 * @param bufferLength Number of bytes to read.
 * @return ESP_OK on success, ESP_FAIL on SPI communication error.
 */
esp_err_t nrf24l01plus_read_buffer(uint8_t reg_addr, uint8_t* buffer, int buffer_len);

/**
 * Attempts to read a received packet from the RX FIFO.
 *
 * Reads STATUS to check for RX_DR, retrieves the packet if available,
 * flushes the RX FIFO, and clears the RX_DR flag.
 *
 * @param rxBuffer Buffer to store the received payload.
 * @return ESP_OK when a packet is received,
 *         ESP_ERR_NOT_FOUND when no packet is available,
 *         ESP_FAIL on error.
 */
int nrf24l01plus_recieve_packet(uint8_t* rx_buf);

/**
 * Sends a TX payload through the NRF24L01+.
 *
 * Switches to TX mode, loads payload, pulses CE for transmission,
 * and returns to RX mode when interrupts are not used.
 *
 * @param txBuffer Payload data to send.
 * @return ESP_OK on success, ESP_FAIL if transmission fails.
 */
esp_err_t nrf24l01plus_send_packet(uint8_t* tx_buf);

/**
 * Sets the NRF24L01+ module to RX mode.
 *
 * Updates the CONFIG register based on IRQ configuration,
 * asserts CE high to start listening.
 *
 * @return ESP_OK on success, ESP_FAIL on register write error.
 */
esp_err_t nrf24l01plus_receive_mode(void);

/**
 * Sets the NRF24L01+ module to TX mode.
 *
 * Clears PRIM_RX, updates CONFIG register, leaves CE low
 * until the caller triggers a CE pulse for transmission.
 *
 * @return ESP_OK on success, ESP_FAIL on register write error.
 */
esp_err_t nrf24l01plus_send_mode(void);

/**
 * Returns whether the TX FIFO is empty by reading FIFO_STATUS.
 *
 * @return 1 if empty, 0 otherwise.
 */
int nrf24l01plus_txFifoEmpty(void);

/**
 * Returns whether the RX FIFO is empty by reading FIFO_STATUS.
 *
 * @return 1 if empty, 0 otherwise.
 */
int nrf24l01plus_rxFifoEmpty(void);

/**
 * Sends the FLUSH_TX command to clear all entries in the TX FIFO.
 *
 * @return ESP_OK on success, ESP_FAIL on communication error.
 */
esp_err_t nrf24l01plus_flush_tx(void);

/**
 * Sends the FLUSH_RX command to clear all entries in the RX FIFO.
 *
 * @return ESP_OK on success, ESP_FAIL on communication error.
 */
esp_err_t nrf24l01plus_flush_rx(void);

#endif
