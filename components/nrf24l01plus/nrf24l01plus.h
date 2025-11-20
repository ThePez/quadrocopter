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
 * @brief Initialize the radio module and start all related tasks.
 *
 * Sets up the NRF24L01+ radio module by initializing the hardware,
 * clearing its status flags, creating the shared event group,
 * and launching the control, receiver, and transmitter tasks.
 *
 * @param spiMutex Pointer to the SPI bus mutex for safe SPI access.
 * @param spiHost  The SPI host device connected to the radio.
 */
void radio_module_init(SemaphoreHandle_t* spiMutex, spi_host_device_t spiHost);

/**
 * @brief Initializes the SPI device interface for the NRF24L01+ module.
 *
 * Configures SPI parameters such as clock speed, SPI mode (CPOL=0, CPHA=0),
 * and chip select pin. Adds the device to the specified SPI bus.
 *
 * @param spiBus The SPI bus (e.g., HSPI_HOST) to which the device is attached.
 */
esp_err_t nrf24l01plus_spi_init(spi_host_device_t spi_bus);

/**
 * @brief Sets up an external interrupt on the NRF24L01+ IRQ pin.
 *
 * Configures the GPIO pin as input with pull-up and attaches an interrupt
 * handler to trigger on falling edge. Installs the ISR service if not already installed.
 *
 * @param handler Pointer to the ISR handler function to call on interrupt.
 */
esp_err_t nrf24l01plus_interrupt_init(void* handler);

/**
 * @brief Fully initializes the NRF24L01+ module.
 *
 * Sets CE pin, configures optional IRQ interrupt, initializes SPI,
 * sets addresses, disables auto acknowledgment, enables Pipe 0,
 * sets payload size, channel, power and data rate, and enters RX mode.
 *
 * @param spiBus SPI bus to use for communication.
 * @param handler Optional ISR handler function for IRQ pin (can be NULL).
 */
esp_err_t nrf24l01plus_init(spi_host_device_t spi_bus, void* handler);

/**
 * @brief Sends a single-byte command to the NRF24L01+
 *
 * @param command Command byte to send
 */
esp_err_t nrf24l01plus_send_command(uint8_t command);

/**
 * @brief Writes a single byte to an NRF24L01+ register.
 *
 * @param regAddress Address of the register to write to.
 * @param val Value to write into the register.
 */
esp_err_t nrf24l01plus_write_register(uint8_t reg_addr, uint8_t val);

/**
 * @brief Reads a single byte from an NRF24L01+ register.
 *
 * @param regAddress Address of the register to read.
 * @return Value read from the register.
 */
uint8_t nrf24l01plus_read_register(uint8_t reg_addr);

/**
 * @brief Writes a buffer of bytes to an NRF24L01+ register.
 *
 * Typically used for writing TX/RX addresses or payload data.
 *
 * @param regAddress Register address to start writing at.
 * @param buffer Pointer to the data buffer to send.
 * @param bufferLength Number of bytes to write.
 */
esp_err_t nrf24l01plus_write_buffer(uint8_t reg_addr, uint8_t* buffer, int buffer_len);

/**
 * @brief Reads multiple bytes from an NRF24L01+ register.
 *
 * Typically used for reading payload data from RX FIFO.
 *
 * @param regAddress Register address to read from.
 * @param buffer Buffer to store the received data.
 * @param bufferLength Number of bytes to read.
 */
esp_err_t nrf24l01plus_read_buffer(uint8_t reg_addr, uint8_t* buffer, int buffer_len);

/**
 * @brief Attempts to receive a packet from the NRF24L01+ RX FIFO.
 *
 * If a packet is available, reads it into the provided buffer,
 * flushes the RX FIFO, and clears the RX_DR interrupt flag.
 *
 * @param rxBuffer Buffer to store the received payload.
 * @return 1 if a packet was received, 0 otherwise.
 */
int nrf24l01plus_recieve_packet(uint8_t* rx_buf);

/**
 * @brief Sends a data packet using the NRF24L01+.
 *
 * Switches to TX mode, writes the payload, pulses CE to transmit,
 * and returns to RX mode if not using IRQ interrupts.
 *
 * @param txBuffer Pointer to the data buffer to transmit.
 */
esp_err_t nrf24l01plus_send_packet(uint8_t* tx_buf);

/**
 * @brief Puts the NRF24L01+ module into receive (RX) mode.
 *
 * Sets PRIM_RX and PWR_UP bits in CONFIG register and enables CE.
 * Behavior may vary depending on whether IRQs are enabled.
 */
esp_err_t nrf24l01plus_receive_mode(void);

/**
 * @brief Puts the NRF24L01+ module into transmit (TX) mode.
 *
 * Clears PRIM_RX bit, keeps CE low until data is sent.
 * Behavior may vary depending on whether IRQs are enabled.
 */
esp_err_t nrf24l01plus_send_mode(void);

/**
 * @brief Checks whether the TX FIFO is empty.
 *
 * @return 1 if empty, 0 if it contains data.
 */
int nrf24l01plus_txFifoEmpty(void);

/**
 * @brief Checks whether the RX FIFO is empty.
 *
 * @return 1 if empty, 0 if it contains unread data.
 */
int nrf24l01plus_rxFifoEmpty(void);

/**
 * @brief Clears the tx FIFO.
 */
esp_err_t nrf24l01plus_flush_tx(void);

/**
 * @brief Clears the rx FIFO.
 */
esp_err_t nrf24l01plus_flush_rx(void);

#endif
