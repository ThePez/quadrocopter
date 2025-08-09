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
#include "esp_log.h"
#include "esp_rom_sys.h"

#include <stdio.h>
#include <string.h>

// Default radio channel (0–125 range on NRF24L01+)
#define DEFAULT_RF_CHANNEL 40

#define CHECK_ERR(code, msg)                                                                                           \
    do {                                                                                                               \
        esp_err_t err = (code);                                                                                        \
        if (err != ESP_OK) {                                                                                           \
            ESP_LOGE(TAG, msg);                                                                                        \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

#define CHECK_ERR_NO_LOG(code)                                                                                         \
    do {                                                                                                               \
        esp_err_t err = (code);                                                                                        \
        if (err != ESP_OK) {                                                                                           \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

// Default 5-byte address (MSB first)
static uint8_t defaultAddress[] = {0x12, 0x34, 0x56, 0x78, 0x90};
// SPI handle for the NRF24L01+ device
static spi_device_handle_t nrf24l01SpiHandle;
// Flag to indicate if IRQ pin and interrupts are used (0 = no IRQ)
static uint8_t useIQR = 0;
// Tag used for ESP log messages
static const char* TAG = "NRF24L01plus";

esp_err_t nrf24l01plus_spi_init(spi_host_device_t spiBus) {

    // Setup the SPI for the radio module
    spi_device_interface_config_t deviceConfig = {
        .clock_speed_hz = 2000000,           // 1 MHz Clock speed
        .spics_io_num = NRF24L01PLUS_CS_PIN, // Chip Select pin for device
        .queue_size = 1,                     // Number of pending transactions allowed
        .mode = 0 /* SPI mode, representing a pair of (CPOL, CPHA). CPOL = 0 (clock idles low)
                    CPHA = 0 (data is sampled on the rising edge, changed on the falling edge) */
    };

    return spi_bus_add_device(spiBus, &deviceConfig, &nrf24l01SpiHandle);
}

esp_err_t nrf24l01plus_interrupt_init(void* handler) {

    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT,                        // Direction of the pin
        .pin_bit_mask = (1ULL << NRF24L01PLUS_IQR_PIN), // Pin is listed here as a bit mask
        .pull_up_en = GPIO_PULLUP_DISABLE,              // No internal pull-up (IRQ is active low)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,          // No internal pull-down
        .intr_type = GPIO_INTR_NEGEDGE                  // Interrupt on the falling edge
    };

    CHECK_ERR(gpio_config(&config), "GPIO config failed");
    CHECK_ERR(gpio_intr_disable(NRF24L01PLUS_IQR_PIN), "IQR interrupt disable failed");
    // Install the ISR service if not already installed
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ISR install service failed");
        return err;
    }

    // Hook ISR handler for the IQR pin
    CHECK_ERR(gpio_isr_handler_add(NRF24L01PLUS_IQR_PIN, handler, NULL), "ISR handler add failed");
    return ESP_OK;
}

esp_err_t nrf24l01plus_init(spi_host_device_t spiBus, void* handler) {

    // Setup the NRF24L01plus CE pin
    CHECK_ERR(gpio_set_direction(NRF24L01PLUS_CE_PIN, GPIO_MODE_INPUT_OUTPUT), "CE pin setup failed");
    // Configure the GPIO pin used by the IQR pin if a ISR handler passed in
    if (handler != NULL) {
        useIQR = 1;
        CHECK_ERR(nrf24l01plus_interrupt_init(handler), "Interrupt setup failed");
    }

    // Configure the SPI
    CHECK_ERR(nrf24l01plus_spi_init(spiBus), "Spi setup failed");
    // Set CE low for idle state
    NRF_CE_LOW();

    // Power on the IC
    CHECK_ERR(nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x7A), "Power-up failed");
    esp_rom_delay_us(4500); // 4.5 ms spin delay
    // Writes TX_Address to nRF24L01plus
    CHECK_ERR(nrf24l01plus_write_buffer(NRF24L01PLUS_TX_ADDR, defaultAddress, 5), "TX address write failed");
    // Writes RX_Address to nRF24L01
    CHECK_ERR(nrf24l01plus_write_buffer(NRF24L01PLUS_RX_ADDR_P0, defaultAddress, 5), "RX address write failed");
    // Disable Auto.Ack
    CHECK_ERR(nrf24l01plus_write_register(NRF24L01PLUS_EN_AA, 0x00), "Auto-ack disable failed");
    // Enable Pipe0
    CHECK_ERR(nrf24l01plus_write_register(NRF24L01PLUS_EN_RXADDR, 0x01), "Pipe0 enable failed");
    // Select same RX payload width as TX Payload width
    CHECK_ERR(nrf24l01plus_write_register(NRF24L01PLUS_RX_PW_P0, NRF24L01PLUS_TX_PLOAD_WIDTH),
              "RX payload width setup failed");
    // Select RF channel
    CHECK_ERR(nrf24l01plus_write_register(NRF24L01PLUS_RF_CH, DEFAULT_RF_CHANNEL), "RF channel setup failed");
    // TX_PWR:0dBm, Datarate:1Mbps
    CHECK_ERR(nrf24l01plus_write_register(NRF24L01PLUS_RF_SETUP, 0x06), "RF setup failed");
    // Clear Any interrupt bits
    CHECK_ERR(nrf24l01plus_write_register(NRF24L01PLUS_STATUS, 0x70), "Status register clear failed");
    // Put device into receive mode
    CHECK_ERR_NO_LOG(nrf24l01plus_receive_mode());
    return ESP_OK;
}

esp_err_t nrf24l01plus_send_command(uint8_t command) {
    spi_transaction_t transation = {
        .length = 8,           // Transation length in bits
        .tx_buffer = &command, // Pointer to transmit buffer
        .rx_buffer = NULL,     // Pointer to receive buffer
    };

    esp_err_t err = spi_device_transmit(nrf24l01SpiHandle, &transation);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Command transmission failed: 0x%02X", command);
    }

    return err;
}

esp_err_t nrf24l01plus_write_register(uint8_t regAddress, uint8_t val) {

    uint8_t txBuffer[2] = {NRF24L01PLUS_WRITE_REG | regAddress, val};
    spi_transaction_t transaction = {
        .length = 16,          // Transaction length in bits
        .tx_buffer = txBuffer, // Pointer to transmit buffer
        .rx_buffer = NULL,     // Pointer to receive buffer
    };

    return spi_device_transmit(nrf24l01SpiHandle, &transaction);
}

uint8_t nrf24l01plus_read_register(uint8_t regAddress) {

    uint8_t txBuffer[2] = {regAddress, 0xFF};
    uint8_t rxBuffer[2];
    spi_transaction_t transaction = {
        .length = 16,          // Transaction length in bits
        .tx_buffer = txBuffer, // Pointer to transmit buffer
        .rx_buffer = rxBuffer, // Pointer to receive buffer
    };

    esp_err_t err = spi_device_transmit(nrf24l01SpiHandle, &transaction);
    if (err != ESP_OK) {
        return 0; // Return 0 on error
    }

    return rxBuffer[1];
}

esp_err_t nrf24l01plus_write_buffer(uint8_t regAddress, uint8_t* buffer, int bufferLength) {

    uint8_t txBuffer[1 + bufferLength];
    txBuffer[0] = NRF24L01PLUS_WRITE_REG | regAddress;
    memcpy(&txBuffer[1], buffer, bufferLength);

    spi_transaction_t transaction = {
        .length = (bufferLength + 1) * 8, // Transaction length in bits
        .tx_buffer = txBuffer,            // Pointer to transmit buffer
        .rx_buffer = NULL,                // Pointer to receive buffer
    };

    return spi_device_transmit(nrf24l01SpiHandle, &transaction);
}

esp_err_t nrf24l01plus_read_buffer(uint8_t regAddress, uint8_t* buffer, int bufferLength) {

    uint8_t txBuffer[1 + bufferLength];
    txBuffer[0] = regAddress;                 // Registers Address
    memset(&txBuffer[1], 0xFF, bufferLength); // Dummy data
    uint8_t rxBuffer[1 + bufferLength];

    spi_transaction_t transaction = {
        .length = (bufferLength + 1) * 8, // Transaction length in bits
        .tx_buffer = txBuffer,            // Pointer to transmit buffer
        .rx_buffer = rxBuffer             // Pointer to recieve buffer
    };

    esp_err_t err = spi_device_transmit(nrf24l01SpiHandle, &transaction);
    memcpy(buffer, &rxBuffer[1], bufferLength); // Skip the status byte in position 0
    return err;
}

int nrf24l01plus_recieve_packet(uint8_t* rxBuffer) {

    uint8_t status = nrf24l01plus_read_register(NRF24L01PLUS_STATUS); // Read STATUS register
    if (status & NRF24L01PLUS_RX_DR) {                                // Data Ready RX FIFO interrupt triggered

        esp_err_t err = nrf24l01plus_read_buffer(NRF24L01PLUS_RD_RX_PLOAD, rxBuffer, NRF24L01PLUS_TX_PLOAD_WIDTH);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read RX payload");
            return ESP_FAIL; // Error occurred
        }

        err = nrf24l01plus_flush_rx();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to flush RX");
            return ESP_FAIL; // Error occurred
        }

        err = nrf24l01plus_write_register(NRF24L01PLUS_STATUS, NRF24L01PLUS_RX_DR); // Clear RX_DR
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear RX_DR flag");
            return ESP_FAIL; // Error occurred
        }
        return ESP_OK + 1; // Packet received successfully
    }

    return ESP_OK; // No packet available
}

esp_err_t nrf24l01plus_send_packet(uint8_t* txBuffer) {

    CHECK_ERR_NO_LOG(nrf24l01plus_send_mode());

    CHECK_ERR(nrf24l01plus_write_buffer(NRF24L01PLUS_WR_TX_PLOAD, txBuffer, NRF24L01PLUS_TX_PLOAD_WIDTH),
              "Failed to write TX payload");

    NRF_CE_LOW();
    esp_rom_delay_us(20);
    NRF_CE_HIGH(); // CE pulse >10 µs
    esp_rom_delay_us(20);
    NRF_CE_LOW();

    if (!useIQR) {
        esp_rom_delay_us(200); // Wait for TX -> Standby-I (~200 us)
        CHECK_ERR_NO_LOG(nrf24l01plus_receive_mode());
    }

    return ESP_OK;
}

esp_err_t nrf24l01plus_receive_mode(void) {

    esp_err_t err;
    if (useIQR) {
        err = nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x1B);
    } else {
        err = nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x7B);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure receive mode");
        return err;
    }

    NRF_CE_HIGH();
    return ESP_OK;
}

esp_err_t nrf24l01plus_send_mode(void) {

    NRF_CE_LOW();

    esp_err_t err;
    if (useIQR) {
        err = nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x1A);
    } else {
        err = nrf24l01plus_write_register(NRF24L01PLUS_CONFIG, 0x7A);
    }

    if (err != ESP_OK) {
        NRF_CE_HIGH();
        ESP_LOGE(TAG, "Failed to configure send mode");
    }

    return err;
}

int nrf24l01plus_txFifoEmpty(void) {

    uint8_t fifoStatus;
    fifoStatus = nrf24l01plus_read_register(NRF24L01PLUS_FIFO_STATUS);
    return !!(fifoStatus & NRF24L01PLUS_FIFO_TX_EMPTY);
}

int nrf24l01plus_rxFifoEmpty(void) {

    uint8_t fifoStatus;
    fifoStatus = nrf24l01plus_read_register(NRF24L01PLUS_FIFO_STATUS);
    return !!(fifoStatus & NRF24L01PLUS_FIFO_RX_EMPTY);
}

esp_err_t nrf24l01plus_flush_tx(void) {
    return nrf24l01plus_send_command(NRF24L01PLUS_FLUSH_TX);
}

esp_err_t nrf24l01plus_flush_rx(void) {
    return nrf24l01plus_send_command(NRF24L01PLUS_FLUSH_RX);
}