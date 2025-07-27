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

// Default radio channel (0–125 range on NRF24L01+)
#define DEFAULT_RF_CHANNEL 40

// Default 5-byte address (MSB first)
static uint8_t defaultAddress[] = {0x12, 0x34, 0x56, 0x78, 0x90};
// SPI handle for the NRF24L01+ device
static spi_device_handle_t nrf24l01SpiHandle;
// Flag to indicate if IRQ pin and interrupts are used (0 = no IRQ)
static uint8_t useIQR = 0;

/**
 * @brief Initializes the SPI device interface for the NRF24L01+ module.
 *
 * Configures SPI parameters such as clock speed, SPI mode (CPOL=0, CPHA=0),
 * and chip select pin. Adds the device to the specified SPI bus.
 *
 * @param spiBus The SPI bus (e.g., HSPI_HOST) to which the device is attached.
 */
void nrf24l01plus_spi_init(spi_host_device_t spiBus) {

    // Setup the SPI for the radio module
    spi_device_interface_config_t deviceConfig = {
        .clock_speed_hz = 2000000,           // 1 MHz Clock speed
        .spics_io_num = NRF24L01PLUS_CS_PIN, // Chip Select pin for device
        .queue_size = 1,                     // Number of pending transactions allowed
        .mode = 0 /* SPI mode, representing a pair of (CPOL, CPHA). CPOL = 0 (clock idles low)
                    CPHA = 0 (data is sampled on the rising edge, changed on the falling edge) */
    };

    ESP_ERROR_CHECK(spi_bus_add_device(spiBus, &deviceConfig, &nrf24l01SpiHandle));
}

/**
 * @brief Sets up an external interrupt on the NRF24L01+ IRQ pin.
 *
 * Configures the GPIO pin as input with pull-up and attaches an interrupt
 * handler to trigger on falling edge. Installs the ISR service if not already installed.
 *
 * @param handler Pointer to the ISR handler function to call on interrupt.
 */
void nrf24l01plus_interrupt_init(void* handler) {

    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT,                        // Direction of the pin
        .pin_bit_mask = (1ULL << NRF24L01PLUS_IQR_PIN), // Pin is listed here as a bit mask
        .pull_up_en = GPIO_PULLUP_DISABLE,              // No internal pull-up (IRQ is active low)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,          // No internal pull-down
        .intr_type = GPIO_INTR_NEGEDGE                  // Interrupt on the falling edge
    };

    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_intr_disable(NRF24L01PLUS_IQR_PIN);

    // Install the ISR service if not already installed
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    // Hook ISR handler for the IQR pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(NRF24L01PLUS_IQR_PIN, handler, NULL));
}

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
void nrf24l01plus_init(spi_host_device_t spiBus, void* handler) {

    // Setup the NRF24L01plus CE pin
    gpio_set_direction(NRF24L01PLUS_CE_PIN, GPIO_MODE_INPUT_OUTPUT);

    // Configure the GPIO pin used by the IQR pin if a ISR handler passed in
    if (handler != NULL) {
        useIQR = 1;
        nrf24l01plus_interrupt_init(handler);
    }

    // Configure the SPI
    nrf24l01plus_spi_init(spiBus);

    // Set CE low for idle state
    NRF_CE_LOW();

    // Power on the IC
    nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x7A);
    esp_rom_delay_us(4500); // 4.5 ms spin delay
    // Writes TX_Address to nRF24L01
    nrf24l01plus_write_buffer(NRF24L01PLUS_TX_ADDR, defaultAddress, 5);
    // Writes RX_Address to nRF24L01
    nrf24l01plus_write_buffer(NRF24L01PLUS_RX_ADDR_P0, defaultAddress, 5);
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
    // Clear Any interrupt bits
    nrf24l01plus_write_register(NRF24L01PLUS_STATUS, 0x70);
    // Put device into receive mode
    nrf24l01plus_receive_mode();
}

/**
 * @brief Writes a single byte to an NRF24L01+ register.
 *
 * @param regAddress Address of the register to write to.
 * @param val Value to write into the register.
 */
void nrf24l01plus_write_register(uint8_t regAddress, uint8_t val) {

    uint8_t txBuffer[2] = {NRF24L01PLUS_WRITE_REG | regAddress, val};
    spi_transaction_t transaction = {
        .length = 16,          // Transaction length in bits
        .tx_buffer = txBuffer, // Pointer to transmit buffer
        .rx_buffer = NULL,     // Pointer to receive buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(nrf24l01SpiHandle, &transaction));
}

/**
 * @brief Reads a single byte from an NRF24L01+ register.
 *
 * @param regAddress Address of the register to read.
 * @return Value read from the register.
 */
uint8_t nrf24l01plus_read_register(uint8_t regAddress) {

    uint8_t txBuffer[2] = {regAddress, 0xFF};
    uint8_t rxBuffer[2];
    spi_transaction_t transaction = {
        .length = 16,          // Transaction length in bits
        .tx_buffer = txBuffer, // Pointer to transmit buffer
        .rx_buffer = rxBuffer, // Pointer to receive buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(nrf24l01SpiHandle, &transaction));
    return rxBuffer[1];
}

/**
 * @brief Writes a buffer of bytes to an NRF24L01+ register.
 *
 * Typically used for writing TX/RX addresses or payload data.
 *
 * @param regAddress Register address to start writing at.
 * @param buffer Pointer to the data buffer to send.
 * @param bufferLength Number of bytes to write.
 */
void nrf24l01plus_write_buffer(uint8_t regAddress, uint8_t* buffer, int bufferLength) {

    uint8_t txBuffer[1 + bufferLength];
    txBuffer[0] = NRF24L01PLUS_WRITE_REG | regAddress;
    memcpy(&txBuffer[1], buffer, bufferLength);

    spi_transaction_t transaction = {
        .length = (bufferLength + 1) * 8, // Transaction length in bits
        .tx_buffer = txBuffer,            // Pointer to transmit buffer
        .rx_buffer = NULL,                // Pointer to receive buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(nrf24l01SpiHandle, &transaction));
}

/**
 * @brief Reads multiple bytes from an NRF24L01+ register.
 *
 * Typically used for reading payload data from RX FIFO.
 *
 * @param regAddress Register address to read from.
 * @param buffer Buffer to store the received data.
 * @param bufferLength Number of bytes to read.
 */

void nrf24l01plus_read_buffer(uint8_t regAddress, uint8_t* buffer, int bufferLength) {

    uint8_t txBuffer[1 + bufferLength];
    txBuffer[0] = regAddress;                 // Registers Address
    memset(&txBuffer[1], 0xFF, bufferLength); // Dummy data
    uint8_t rxBuffer[1 + bufferLength];

    spi_transaction_t transaction = {
        .length = (bufferLength + 1) * 8, // Transaction length in bits
        .tx_buffer = txBuffer,            // Pointer to transmit buffer
        .rx_buffer = rxBuffer             // Pointer to recieve buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(nrf24l01SpiHandle, &transaction));
    memcpy(buffer, &rxBuffer[1], bufferLength); // Skip the status byte in position 0
}

/**
 * @brief Attempts to receive a packet from the NRF24L01+ RX FIFO.
 *
 * If a packet is available, reads it into the provided buffer,
 * flushes the RX FIFO, and clears the RX_DR interrupt flag.
 *
 * @param rxBuffer Buffer to store the received payload.
 * @return 1 if a packet was received, 0 otherwise.
 */
int nrf24l01plus_recieve_packet(uint8_t* rxBuffer) {

    uint8_t status = nrf24l01plus_read_register(NRF24L01PLUS_STATUS); // Read STATUS register
    if (status & NRF24L01PLUS_RX_DR) {                                // Data Ready RX FIFO interrupt triggered

        esp_rom_delay_us(10);
        nrf24l01plus_read_buffer(NRF24L01PLUS_RD_RX_PLOAD, rxBuffer, NRF24L01PLUS_TX_PLOAD_WIDTH);
        esp_rom_delay_us(10);
        nrf24l01plus_write_register(NRF24L01PLUS_FLUSH_RX, 0); // Flush RX FIFO
        esp_rom_delay_us(10);
        nrf24l01plus_write_register(NRF24L01PLUS_STATUS, NRF24L01PLUS_RX_DR); // Clear RX_DR
        esp_rom_delay_us(10);
        return 1;
    }

    return 0;
}

/**
 * @brief Sends a data packet using the NRF24L01+.
 *
 * Switches to TX mode, writes the payload, pulses CE to transmit,
 * and returns to RX mode if not using IRQ interrupts.
 *
 * @param txBuffer Pointer to the data buffer to transmit.
 */
void nrf24l01plus_send_packet(uint8_t* txBuffer) {

    nrf24l01plus_send_mode();
    nrf24l01plus_write_buffer(NRF24L01PLUS_WR_TX_PLOAD, txBuffer, NRF24L01PLUS_TX_PLOAD_WIDTH);
    NRF_CE_LOW();
    esp_rom_delay_us(20);
    NRF_CE_HIGH(); // CE pulse >10 µs
    esp_rom_delay_us(20);
    NRF_CE_LOW();

    if (!useIQR) {
        esp_rom_delay_us(200); // Wait for TX -> Standby-I (~4.5 ms)
        nrf24l01plus_receive_mode();
    }
}

/**
 * @brief Puts the NRF24L01+ module into receive (RX) mode.
 *
 * Sets PRIM_RX and PWR_UP bits in CONFIG register and enables CE.
 * Behavior may vary depending on whether IRQs are enabled.
 */
void nrf24l01plus_receive_mode(void) {

    if (useIQR) {
        nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x1B);
    } else {
        nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x7B);
    }

    NRF_CE_HIGH();
}

/**
 * @brief Puts the NRF24L01+ module into transmit (TX) mode.
 *
 * Clears PRIM_RX bit, keeps CE low until data is sent.
 * Behavior may vary depending on whether IRQs are enabled.
 */
void nrf24l01plus_send_mode(void) {

    NRF_CE_LOW();

    if (useIQR) {
        nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x1A);
    } else {
        nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x7A);
    }
}

/**
 * @brief Checks whether the TX FIFO is empty.
 *
 * @return 1 if empty, 0 if it contains data.
 */
int nrf24l01plus_txFifoEmpty(void) {

    uint8_t fifoStatus;
    fifoStatus = nrf24l01plus_read_register(NRF24L01PLUS_FIFO_STATUS);
    return !!(fifoStatus & NRF24L01PLUS_FIFO_TX_EMPTY);
}

/**
 * @brief Checks whether the RX FIFO is empty.
 *
 * @return 1 if empty, 0 if it contains unread data.
 */
int nrf24l01plus_rxFifoEmpty(void) {

    uint8_t fifoStatus;
    fifoStatus = nrf24l01plus_read_register(NRF24L01PLUS_FIFO_STATUS);
    return !!(fifoStatus & NRF24L01PLUS_FIFO_RX_EMPTY);
}
