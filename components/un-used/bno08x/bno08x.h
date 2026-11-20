/*
 *****************************************************************************
 * File: bno08x.h
 * Author: Jack Cairns
 * Brief: BNO08x IMU driver (SPI), built on the SH2 library in sh2/.
 *****************************************************************************
 */

#ifndef BNO08X_H
#define BNO08X_H

#include <esp_err.h>
#include <driver/spi_master.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Wiring
#define BNO08X_SPI_HOST SPI3_HOST
#define BNO08X_PIN_SCLK 18
#define BNO08X_PIN_MOSI 23 // Sensor "DI"
#define BNO08X_PIN_MISO 19 // Sensor "SDA"
#define BNO08X_PIN_CS 22
#define BNO08X_PIN_INT 21 // Sensor "INT" / H_INTN, active low
#define BNO08X_PIN_RST 33 // Sensor "RST", active low
#define BNO08X_SPI_HZ 2000000

// TODO: public API (init, enable report, service, ...)

#ifdef __cplusplus
}
#endif
#endif
