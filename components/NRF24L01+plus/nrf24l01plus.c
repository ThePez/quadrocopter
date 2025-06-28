/*
 ******************************************************************************
 * File: nrf24l01plus.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 ******************************************************************************
 */

#include "nrf24l01plus.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

#define DEFAULT_RF_CHANNEL 40                                   // Default radio channel
static uint8_t default_addr[] = {0x12, 0x34, 0x56, 0x78, 0x90}; // Default address with MSB first

static spi_device_handle_t radio_spi_handle;

/* nrf24l01plus_spi_init()
 * -----------------------
 *
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
 *
 */
void nrf24l01plus_init(spi_host_device_t spi_bus) {

    // Setup the NRF24L01plus CE pin
    gpio_set_direction(NRF24L01PLUS_CE_PIN, GPIO_MODE_INPUT_OUTPUT);

    // Configure the SPI
    nrf24l01plus_spi_init(spi_bus);

    // Set CE low for idle state
    NRF_CE_LOW();

    // Power down nrf before powering up.
    nrf24l01plus_write_buffer(NRF24L01PLUS_WRITE_REG | NRF24L01PLUS_TX_ADDR, default_addr,
                              5); // Writes TX_Address to nRF24L01
    nrf24l01plus_write_buffer(NRF24L01PLUS_WRITE_REG | NRF24L01PLUS_RX_ADDR_P0, default_addr,
                              5); // NRF24L01P_TX_ADR_WIDTH);

    nrf24l01plus_write_register(NRF24L01PLUS_EN_AA, 0x00);     // Disable Auto.Ack
    nrf24l01plus_write_register(NRF24L01PLUS_EN_RXADDR, 0x01); // Enable Pipe0
    // Select same RX payload width as TX Payload width
    nrf24l01plus_write_register(NRF24L01PLUS_RX_PW_P0, NRF24L01PLUS_TX_PLOAD_WIDTH);

    nrf24l01plus_write_register(NRF24L01PLUS_RF_CH, DEFAULT_RF_CHANNEL); // Select RF channel
    nrf24l01plus_write_register(NRF24L01PLUS_RF_SETUP, 0x06);            // TX_PWR:0dBm, Datarate:1Mbps

    // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
    nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x02);
}

/* nrf24l01plus_write_register()
 * -----------------------------
 *
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
 *
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
 *
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
 *
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
 *
 */
int nrf24l01plus_recieve_packet(uint8_t* rx_buf) {

    uint8_t status = nrf24l01plus_read_register(NRF24L01PLUS_STATUS); // read register STATUS's value

    if (status & NRF24L01PLUS_RX_DR) { // Check if Data Ready RX FIFO interrupt triggered
        // esp_rom_delay_us(10);
        nrf24l01plus_read_buffer(NRF24L01PLUS_RD_RX_PLOAD, rx_buf,
                                 NRF24L01PLUS_TX_PLOAD_WIDTH); // read playload to rx_buf
        // esp_rom_delay_us(10);
        nrf24l01plus_write_register(NRF24L01PLUS_FLUSH_RX, 0); // clear RX_FIFO. Writes CMD then dummy byte
        // esp_rom_delay_us(10);
        NRF_CE_LOW();
        nrf24l01plus_write_register(NRF24L01PLUS_STATUS,
                                    0x70); // clear RX_DR or TX_DS or MAX_RT interrupt flag. Writing 1 clears bit
        return 1;
    }

    return 0;
}

/* nrf24l01plus_send_packet()
 * --------------------------
 *
 */
void nrf24l01plus_send_packet(uint8_t* tx_buf) {

    nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x72); // Set PWR_UP bit, disable CRC(2 unsigned chars) & Prim:TX.
    nrf24l01plus_write_buffer(NRF24L01PLUS_WR_TX_PLOAD, tx_buf,
                              NRF24L01PLUS_TX_PLOAD_WIDTH); // write playload to TX_FIFO

    NRF_CE_LOW();
    esp_rom_delay_us(20);
    NRF_CE_HIGH();
    esp_rom_delay_us(20);
    NRF_CE_LOW();
    vTaskDelay(pdMS_TO_TICKS(4.5)); // Transition TX mode to Standby-I (Takes ~4.5ms)
    nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x73);
    NRF_CE_HIGH();
}

/* nrf24l01plus_ce()
 * -----------------
 *
 */
void nrf24l01plus_ce(void) {

    nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x03); // Set PWR_UP bit, enable CRC(2 unsigned chars) &
    NRF_CE_HIGH();                                          // Set CE pin high to enable RX device
}

/* nrf24l01plus_cepin()
 * --------------------
 *
 */
void nrf24l01plus_cepin(void) {
    NRF_CE_HIGH();
    // Put the CE Pin high to enable reception of radio packets
    // HAL_GPIO_WritePin(NRF_MODE_GPIO_PORT, NRF_MODE_PIN, 1);
}

/* nrf24l01plus_txFifoEmpty()
 * --------------------------
 *
 */
int nrf24l01plus_txFifoEmpty(void) {

    uint8_t fifoStatus;
    fifoStatus = nrf24l01plus_read_register(NRF24L01PLUS_FIFO_STATUS);
    return (fifoStatus & NRF24L01PLUS_FIFO_TX_EMPTY);
}

/* nrf24l01plus_rxFifoEmpty()
 * --------------------------
 *
 */
int nrf24l01plus_rxFifoEmpty(void) {

    uint8_t fifoStatus;
    fifoStatus = nrf24l01plus_read_register(NRF24L01PLUS_FIFO_STATUS);
    return (fifoStatus & NRF24L01PLUS_FIFO_RX_EMPTY);
}
