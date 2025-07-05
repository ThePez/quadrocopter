/*
 ******************************************************************************
 * File: lis3DH.h
 * Author: Jack Cairns
 * Date: 01-07-2025
 * Brief: Driver for the LIS3DH accelerometer
 * REFERENCE: None
 ******************************************************************************
 */

#ifndef LIS3DH_H
#define LIS3DH_H

#include <stdint.h>

#include "driver/spi_master.h"

#define LIS3DH_CS_PIN 25

// Setup constants
#define SPIN_TIME 10000
#define WHO_AM_I_CONTENTS 0x33

// Bits to set for SPI Read/Write
#define LIS3DH_SPI_READ 0x80
#define LIS3DH_SPI_WRITE 0x7F
// Auto-increment for multi-byte read
#define LIS3DH_SPI_MULTI_READ 0xC0
//Axis Data length
#define LIS3DH_AXIS_DATA_LENGTH 6

// Control Register 1 bits
#define CTRL_REG1_ODR3 0x80
#define CTRL_REG1_ODR2 0x40
#define CTRL_REG1_ODR1 0x20
#define CTRL_REG1_ODR0 0x10
#define CTRL_REG1_LPEN 0x08
#define CTRL_REG1_ZEN 0x04
#define CTRL_REG1_YEN 0x02
#define CTRL_REG1_XEN 0x01

// Polling rates
#define RATE_1_HZ 0x10
#define RATE_10_HZ 0x20
#define RATE_25_HZ 0x30
#define RATE_50_HZ 0x40
#define RATE_100_HZ 0x50
#define RATE_200_HZ 0x60
#define RATE_400_HZ 0x70

// Device Registers
#define LIS3DH_STATUS_REG_AUX 0x07
#define LIS3DH_WHO_AM_I 0x0F
#define LIS3DH_TEMP_CFG_REG 0x1F
#define LIS3DH_CTRL_REG1 0x20 // power state, enable each axis
#define LIS3DH_CTRL_REG2 0x21 // high pass filter settings -- not useful for us.
#define LIS3DH_CTRL_REG3 0x22 // Set interrupt source.
#define LIS3DH_CTRL_REG4 0x23 // along with other things sets up max accel scale
#define LIS3DH_CTRL_REG5 0x24 // Set up 6D/4D interrupts.
#define LIS3DH_CTRL_REG6 0x25
#define LIS3DH_REFERENCE 0x26
#define LIS3DH_STATUS_REG2 0x27

// Axis Registers
#define LIS3DH_OUT_X_L 0x28
#define LIS3DH_OUT_X_H 0x29
#define LIS3DH_OUT_Y_L 0x2A
#define LIS3DH_OUT_Y_H 0x2B
#define LIS3DH_OUT_Z_L 0x2C
#define LIS3DH_OUT_Z_H 0x2D

// 16 bit range / 2G
#define SENSITIVITY 16384.0

// Status Enum
typedef enum {
    LISSUCCESS,
    LISWHOERROR,
    LISNOTSUPPORTED,
    LISGENERICERROR,
    LISOUTOFBOUNDS,
    LISALLONESWARNING
} LisErrorCode;

///////////////////////////// Function Prototypes /////////////////////////////

void lis3dh_spi_init(spi_host_device_t spi_bus);
void lis3dh_init(spi_host_device_t spi_bus);
void lisPowerDown(void);
uint8_t lisReadRegister(uint8_t addr);
void lisWriteRegister(uint8_t addr, uint8_t data);
void lisReadAxisData(int16_t* x, int16_t* y, int16_t* z);
double getPitchAngle(void);
double getRollAngle(void);

#endif
