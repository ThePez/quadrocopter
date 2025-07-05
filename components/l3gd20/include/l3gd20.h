/*
 ******************************************************************************
 * File: l3gd20.h
 * Author: Jack Cairns
 * Date: 02-07-2025
 * Brief: Driver for the L3GD20 gyroscope
 * REFERENCE: None
 ******************************************************************************
 */

#ifndef L3GD20_H
#define L3GD20_H

#include <stdint.h>

#include "driver/spi_master.h"

// SPI CS pin
#define GYRO_CS_PIN 24

// Setup constants
#define GYRO_WHO_AM_I_CONTENTS 0xD4

// Bits to set for SPI Read/Write
#define GYRO_SPI_READ 0x80
#define GYRO_SPI_WRITE 0x7F
// Auto-increment for multi-byte read
#define GYRO_SPI_MULTI_READ 0xC0
// Axis Data length
#define GYRO_AXIS_DATA_LENGTH 6

// Control Register 1 bits
#define GYRO_CTRL_REG1_ODR3 0x80
#define GYRO_CTRL_REG1_ODR2 0x40
#define GYRO_CTRL_REG1_ODR1 0x20
#define GYRO_CTRL_REG1_ODR0 0x10
#define GYRO_CTRL_REG1_LPEN 0x08
#define GYRO_CTRL_REG1_ZEN 0x04
#define GYRO_CTRL_REG1_YEN 0x02
#define GYRO_CTRL_REG1_XEN 0x01

// Polling rates
#define GYRO_RATE_1_HZ 0x10
#define GYRO_RATE_10_HZ 0x20
#define GYRO_RATE_25_HZ 0x30
#define GYRO_RATE_50_HZ 0x40
#define GYRO_RATE_100_HZ 0x50
#define GYRO_RATE_200_HZ 0x60
#define GYRO_RATE_400_HZ 0x70

// Device Registers
#define GYRO_STATUS_REG_AUX 0x07
#define GYRO_WHO_AM_I 0x0F
#define GYRO_TEMP_CFG_REG 0x1F
#define GYRO_CTRL_REG1 0x20 // power state, enable each axis
#define GYRO_CTRL_REG2 0x21 // high pass filter settings -- not useful for us.
#define GYRO_CTRL_REG3 0x22 // Set interrupt source.
#define GYRO_CTRL_REG4 0x23 // along with other things sets up max accel scale
#define GYRO_CTRL_REG5 0x24 // Set up 6D/4D interrupts.
#define GYRO_CTRL_REG6 0x25
#define GYRO_REFERENCE 0x26
#define GYRO_STATUS_REG2 0x27

// Axis Registers
#define GYRO_OUT_X_L 0x28
#define GYRO_OUT_X_H 0x29
#define GYRO_OUT_Y_L 0x2A
#define GYRO_OUT_Y_H 0x2B
#define GYRO_OUT_Z_L 0x2C
#define GYRO_OUT_Z_H 0x2D

// 16 bit range / 2G
#define SENSITIVITY 16384.0

// Status Enum
typedef enum {
    GYROSUCCESS,
    GYROWHOERROR,
    GYRONOTSUPPORTED,
    GYROGENERICERROR,
    GYROOUTOFBOUNDS,
    GYROALLONESWARNING
} GyroErrorCode;

///////////////////////////// Function Prototypes /////////////////////////////

void gyro_spi_init(spi_host_device_t spi_bus);
void gyro_init(spi_host_device_t spi_bus);
void gyroPowerDown(void);
uint8_t gyroReadRegister(uint8_t addr);
void gyroWriteRegister(uint8_t addr, uint8_t data);
void readAxisData(int16_t* x, int16_t* y, int16_t* z);
double getPitchAngle(void);
double getRollAngle(void);


#endif
