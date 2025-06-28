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

#include <stdint.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define NRF24L01PLUS_CS_PIN 5
#define NRF24L01PLUS_CE_PIN 6

#define NRF24L01PLUS_TX_ADR_WIDTH 5    // 5 unsigned chars TX(RX) address width
#define NRF24L01PLUS_TX_PLOAD_WIDTH 32 // 32 unsigned chars TX payload

// SPI(nRF24L01) commands
#define NRF24L01PLUS_READ_REG 0x00        // Define read command to register
#define NRF24L01PLUS_WRITE_REG 0x20       // Define write command to register
#define NRF24L01PLUS_RD_RX_PLOAD 0x61     // Define RX payload register address
#define NRF24L01PLUS_WR_TX_PLOAD 0xA0     // Define TX payload register address
#define NRF24L01PLUS_FLUSH_TX 0xE1        // Define flush TX register command
#define NRF24L01PLUS_FLUSH_RX 0xE2        // Define flush RX register command
#define NRF24L01PLUS_ACTIVATE 0x50        // ACTIVATE additional features
#define NRF24L01PLUS_REUSE_TX_PL 0xE3     // Define reuse TX payload register command
#define NRF24L01PLUS_R_RX_PL_WID 0x60     // Define Read RX-payload width command
#define NRF24L01PLUS_W_ACK_PAYLOAD 0xA8   // Write payload to be used in ACK packet on pipe PPP
#define NRF24L01PLUS_W_TX_PAYLOAD_NOACK 0xB0 // Used in TX mode, Disable AUTOACK on this specific packet
#define NRF24L01PLUS_OP_NOP 0xFF             // Define No Operation, might be used to read status register

// SPI(nRF24L01) registers(addresses)
#define NRF24L01PLUS_CONFIG 0x00   // 'Config' register address
#define NRF24L01PLUS_EN_AA 0x01    // 'Enable Auto Acknowledgment' register address
#define NRF24L01PLUS_EN_RXADDR 0x02 // 'Enabled RX addresses' register address
#define NRF24L01PLUS_SETUP_AW 0x03  // 'Setup address width' register address
#define NRF24L01PLUS_SETUP_RETR 0x04 // 'Setup Auto. Retrans' register address
#define NRF24L01PLUS_RF_CH 0x05      // 'RF channel' register address
#define NRF24L01PLUS_RF_SETUP 0x06   // 'RF setup' register address
#define NRF24L01PLUS_STATUS 0x07     // 'Status' register address
#define NRF24L01PLUS_OBSERVE_TX 0x08 // 'Observe TX' register address
#define NRF24L01PLUS_RPD 0x09        // 'Carrier Detect' register address
#define NRF24L01PLUS_RX_ADDR_P0 0x0A // 'RX address pipe0' register address
#define NRF24L01PLUS_RX_ADDR_P1 0x0B // 'RX address pipe1' register address
#define NRF24L01PLUS_RX_ADDR_P2 0x0C // 'RX address pipe2' register address
#define NRF24L01PLUS_RX_ADDR_P3 0x0D // 'RX address pipe3' register address
#define NRF24L01PLUS_RX_ADDR_P4 0x0E // 'RX address pipe4' register address
#define NRF24L01PLUS_RX_ADDR_P5 0x0F // 'RX address pipe5' register address
#define NRF24L01PLUS_TX_ADDR 0x10  // 'TX address' register address
#define NRF24L01PLUS_RX_PW_P0 0x11 // 'RX payload width, pipe0' register address
#define NRF24L01PLUS_RX_PW_P1 0x12 // 'RX payload width, pipe1' register address
#define NRF24L01PLUS_RX_PW_P2 0x13 // 'RX payload width, pipe2' register address
#define NRF24L01PLUS_RX_PW_P3 0x14 // 'RX payload width, pipe3' register address
#define NRF24L01PLUS_RX_PW_P4 0x15 // 'RX payload width, pipe4' register address
#define NRF24L01PLUS_RX_PW_P5 0x16 // 'RX payload width, pipe5' register address
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

void nrf24l01plus_spi_init(spi_host_device_t spi_bus);
void nrf24l01plus_init(spi_host_device_t spi_bus);
void nrf24l01plus_write_register(uint8_t reg_addr, uint8_t val);
uint8_t nrf24l01plus_read_register(uint8_t reg_addr);
void nrf24l01plus_write_buffer(uint8_t reg_addr, uint8_t* buffer, int buffer_len);
void nrf24l01plus_read_buffer(uint8_t reg_addr, uint8_t* buffer, int buffer_len);
int nrf24l01plus_recieve_packet(uint8_t* rx_buf);
void nrf24l01plus_send_packet(uint8_t* tx_buf);
void nrf24l01plus_ce(void);
void nrf24l01plus_cepin(void);
int nrf24l01plus_txFifoEmpty(void);
int nrf24l01plus_rxFifoEmpty(void);

#endif