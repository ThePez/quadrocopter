/*
 ******************************************************************************
 * File: nrf24l01plus.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief: Driver for the NRF24L01plus module. Contains various functions 
 * REFERENCE: NRF24L01plus data sheet for timings, register addresses & values
 ******************************************************************************
 */

#include "nrf24l01plus.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include <stdio.h>
#include <string.h>

#define DEFAULT_RF_CHANNEL 40                                   // Default radio channel
static uint8_t default_addr[] = {0x12, 0x34, 0x56, 0x78, 0x90}; // Default address with MSB first
static spi_device_handle_t radio_spi_handle;                    // Handle for the ESP's SPI device driver

/* nrf24l01plus_spi_init()
 * -----------------------
 * Initializes the SPI device handle for the NRF24L01+.
 * Configures the clock speed, SPI mode (CPOL=0, CPHA=0), chip select pin,
 * and adds the device to the specified SPI bus.
 *
 * Parameters:
 *   spi_bus - The SPI bus to use (e.g., HSPI_HOST).
 */
void nrf24l01plus_spi_init(spi_host_device_t spi_bus) {

    // Setup the SPI for the radio module
    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 1000000,           // 1 MHz Clock speed
        .spics_io_num = NRF24L01PLUS_CS_PIN, // Chip Select pin for device
        .queue_size = 1,                     // Number of pending transactions allowed
        .mode = 0 /* SPI mode, representing a pair of (CPOL, CPHA). CPOL = 0 (clock idles low)
                    CPHA = 0 (data is sampled on the rising edge, changed on the falling edge) */
    };

    ESP_ERROR_CHECK(spi_bus_add_device(spi_bus, &device_config, &radio_spi_handle));
}

/* nrf24l01plus_init()
 * -------------------
 * Function to setup the NRF24L01plus for use.
 */
void nrf24l01plus_init(spi_host_device_t spi_bus) {

    // Setup the NRF24L01plus CE pin
    gpio_set_direction(NRF24L01PLUS_CE_PIN, GPIO_MODE_INPUT_OUTPUT);
    // Configure the SPI
    nrf24l01plus_spi_init(spi_bus);

    // Set CE low for idle state
    NRF_CE_LOW();

    // Writes TX_Address to nRF24L01
    nrf24l01plus_write_buffer(NRF24L01PLUS_WRITE_REG | NRF24L01PLUS_TX_ADDR, default_addr, 5);
    // Writes RX_Address to nRF24L01
    nrf24l01plus_write_buffer(NRF24L01PLUS_WRITE_REG | NRF24L01PLUS_RX_ADDR_P0, default_addr, 5);
    // Disable Auto.Ack
    nrf24l01plus_write_register(NRF24L01PLUS_EN_AA, 0x00);
    // Enable Pipe0
    nrf24l01plus_write_register(NRF24L01PLUS_EN_RXADDR, 0x01);
    // Select same RX payload width as TX Payload width
    nrf24l01plus_write_register(NRF24L01PLUS_RX_PW_P0, NRF24L01PLUS_TX_PLOAD_WIDTH);
    // Select RF channel
    nrf24l01plus_write_register(NRF24L01PLUS_RF_CH, DEFAULT_RF_CHANNEL);
    // TX_PWR:0dBm, Datarate:1Mbps
    nrf24l01plus_write_register(NRF24L01PLUS_RF_SETUP, 0x06);

    // Put the device into sending mode
    nrf24l01plus_send_mode();
}

/* nrf24l01plus_write_register()
 * -----------------------------
 * Writes a single byte to a specified NRF24L01+ register.
 *
 * Parameters:
 *   reg_addr - Register address to write to.
 *   val      - Value to write.
 */
void nrf24l01plus_write_register(uint8_t reg_addr, uint8_t val) {

    uint8_t tx_buffer[2] = {NRF24L01PLUS_WRITE_REG | reg_addr, val};
    spi_transaction_t transaction = {
        .length = 16,            // Transaction length in bits
        .tx_buffer = &tx_buffer, // Pointer to transmit buffer
        .rx_buffer = NULL,       // Pointer to receive buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(radio_spi_handle, &transaction));
}

/* nrf24l01plus_read_register()
 * ----------------------------
 * Reads a single byte from a specified NRF24L01+ register.
 *
 * Parameters:
 *   reg_addr - Register address to read from.
 *
 * Returns:
 *   The byte read from the register.
 */
uint8_t nrf24l01plus_read_register(uint8_t reg_addr) {

    uint8_t tx_buffer[2] = {reg_addr, 0xFF};
    uint8_t rx_buffer[2];
    spi_transaction_t transaction = {
        .length = 16,            // Transaction length in bits
        .tx_buffer = &tx_buffer, // Pointer to transmit buffer
        .rx_buffer = rx_buffer,  // Pointer to receive buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(radio_spi_handle, &transaction));
    return rx_buffer[1];
}

/* nrf24l01plus_write_buffer()
 * ---------------------------
 * Writes multiple bytes to a specified NRF24L01+ register.
 * Used for operations like setting TX or RX addresses or sending payloads.
 *
 * Parameters:
 *   reg_addr   - Starting register address to write to.
 *   buffer     - Pointer to the data buffer to write.
 *   buffer_len - Number of bytes to write.
 */
void nrf24l01plus_write_buffer(uint8_t reg_addr, uint8_t* buffer, int buffer_len) {

    uint8_t tx_buffer[1 + buffer_len];
    tx_buffer[0] = NRF24L01PLUS_WRITE_REG | reg_addr;
    memcpy(&tx_buffer[1], buffer, buffer_len);

    spi_transaction_t transaction = {
        .length = (buffer_len + 1) * 8, // Transaction length in bits
        .tx_buffer = &tx_buffer,        // Pointer to transmit buffer
        .rx_buffer = NULL,              // Pointer to receive buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(radio_spi_handle, &transaction));
}

/* nrf24l01plus_read_buffer()
 * --------------------------
 * Reads multiple bytes from a specified NRF24L01+ register.
 * Used for operations like reading received payloads.
 *
 * Parameters:
 *   reg_addr   - Starting register address to read from.
 *   buffer     - Pointer to the buffer to store the read data.
 *   buffer_len - Number of bytes to read.
 */
void nrf24l01plus_read_buffer(uint8_t reg_addr, uint8_t* buffer, int buffer_len) {

    uint8_t tx_buffer[1 + buffer_len];
    tx_buffer[0] = reg_addr;                 // Registers Address
    memset(&tx_buffer[1], 0xFF, buffer_len); // Dummy data
    uint8_t rx_buffer[1 + buffer_len];

    spi_transaction_t transaction = {
        .length = (buffer_len + 1) * 8, // Transaction length in bits
        .tx_buffer = tx_buffer,         // Pointer to transmit buffer
        .rx_buffer = rx_buffer          // Pointer to recieve buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(radio_spi_handle, &transaction));
    memcpy(buffer, &rx_buffer[1], buffer_len); // Skip the status byte in position 0
}

/* nrf24l01plus_recieve_packet()
 * -----------------------------
 * Checks if a new packet has been received (RX_DR bit).
 * If a packet is available:
 *   - Reads the payload from the RX FIFO into the provided buffer.
 *   - Flushes the RX FIFO to clear any leftover data.
 *   - Clears the RX_DR interrupt flag in the STATUS register.
 * Returns:
 *   1 if a packet was read,
 *   0 if no packet was available.
 */
int nrf24l01plus_recieve_packet(uint8_t* rx_buffer) {

    uint8_t status = nrf24l01plus_read_register(NRF24L01PLUS_STATUS); // Read STATUS register
    if (status & NRF24L01PLUS_RX_DR) {                                // Data Ready RX FIFO interrupt triggered

        esp_rom_delay_us(10);
        nrf24l01plus_read_buffer(NRF24L01PLUS_RD_RX_PLOAD, rx_buffer, NRF24L01PLUS_TX_PLOAD_WIDTH);
        esp_rom_delay_us(10);
        nrf24l01plus_write_register(NRF24L01PLUS_FLUSH_RX, 0); // Flush RX FIFO
        esp_rom_delay_us(10);
        nrf24l01plus_write_register(NRF24L01PLUS_STATUS, 0x70); // Clear RX_DR, TX_DS, MAX_RT bits
        return 1;
    }

    return 0;
}

/* nrf24l01plus_send_packet()
 * --------------------------
 * Sends a single payload over the air.
 * Steps:
 *   - Switches the radio to TX mode (PWR_UP = 1, PRIM_RX = 0).
 *   - Writes the new payload into the TX FIFO.
 *   - Pulses the CE pin HIGH for at least 10 µs to start transmission.
 *   - Waits 4.5 ms for the transmission to complete and the radio to return to Standby-I.
 *   - Switches the radio back to RX mode by setting PRIM_RX = 1.
 */
void nrf24l01plus_send_packet(uint8_t* tx_buf) {

    nrf24l01plus_send_mode();
    nrf24l01plus_write_buffer(NRF24L01PLUS_WR_TX_PLOAD, tx_buf, NRF24L01PLUS_TX_PLOAD_WIDTH);
    NRF_CE_LOW();
    esp_rom_delay_us(20);
    NRF_CE_HIGH(); // CE pulse >10 µs
    esp_rom_delay_us(20);
    NRF_CE_LOW();
    esp_rom_delay_us(4500); // Wait for TX -> Standby-I (~4.5 ms)
    nrf24l01plus_recieve_mode();
}

/* nrf24l01plus_recieve_mode()
 * ---------------------------
 * Puts the NRF24L01+ into RX mode.
 * Sets PRIM_RX=1, PWR_UP=1.
 * CONFIG bits: MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO | PWR_UP | PRIM_RX
 * CE high to start listening.
 */
void nrf24l01plus_recieve_mode(void) {

    nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x7B);
    NRF_CE_HIGH();
}

/* nrf24l01plus_send_mode()
 * --------------------
 * Puts the NRF24L01+ into TX mode.
 * Sets PRIM_RX=0, PWR_UP=1.
 * CONFIG bits: MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT | EN_CRC | CRCO | PWR_UP | PRIM_RX
 * CE stays low until you pulse it for TX
 */
void nrf24l01plus_send_mode(void) {

    NRF_CE_LOW();
    nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x7A);
}

/* nrf24l01plus_txFifoEmpty()
 * --------------------------
 * Checks whether the NRF24L01+ TX FIFO is empty.
 *
 * Returns:
 *   Non-zero if the TX FIFO is empty,
 *   0 if the TX FIFO still has data.
 */
int nrf24l01plus_txFifoEmpty(void) {

    uint8_t fifoStatus;
    fifoStatus = nrf24l01plus_read_register(NRF24L01PLUS_FIFO_STATUS);
    return (fifoStatus & NRF24L01PLUS_FIFO_TX_EMPTY);
}

/* nrf24l01plus_rxFifoEmpty()
 * --------------------------
 * Checks whether the NRF24L01+ RX FIFO is empty.
 *
 * Returns:
 *   Non-zero if the RX FIFO is empty,
 *   0 if the RX FIFO has unread data.
 */
int nrf24l01plus_rxFifoEmpty(void) {

    uint8_t fifoStatus;
    fifoStatus = nrf24l01plus_read_register(NRF24L01PLUS_FIFO_STATUS);
    return (fifoStatus & NRF24L01PLUS_FIFO_RX_EMPTY);
}
