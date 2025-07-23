/*
 ******************************************************************************
 * File: lis3DH.c
 * Author: Jack Cairns
 * Date: 01-07-2025
 * Brief: Driver for the LIS3DH accelerometer
 * REFERENCE: None
 ******************************************************************************
 */

#include "lis3DH.h"

#include <math.h>
#include <stdio.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

// Strings for IMU setup
static const char* lis_strings[3] = {"ACCEL Initialization Successful\n", "ACCEL Initialization Failed\n",
                                     "Expected 0x33, got 0x%02X\n"};

// SPI device handle
spi_device_handle_t lis3dh_spi_handle = NULL;

/* lis3dh_spi_init()
 * -----------------------
 * Initializes the SPI device handle for the LIS3DH.
 * Configures the clock speed, SPI mode (CPOL=0, CPHA=0), chip select pin,
 * and adds the device to the specified SPI bus.
 *
 * Parameters:
 *   spi_bus - The SPI bus to use (e.g., HSPI_HOST).
 */
void lis3dh_spi_init(spi_host_device_t spi_bus) {

    // Setup the SPI for the radio module
    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 1000000,     // 1 MHz Clock speed
        .spics_io_num = LIS3DH_CS_PIN, // Chip Select pin for device
        .queue_size = 1,               // Number of pending transactions allowed
        .mode = 0                      /* SPI mode, representing a pair of (CPOL, CPHA). CPOL = 0 (clock idles low)
                                         CPHA = 0 (data is sampled on the rising edge, changed on the falling edge) */
    };

    ESP_ERROR_CHECK(spi_bus_add_device(spi_bus, &device_config, &lis3dh_spi_handle));
}

/* lis3dh_init()
 * -------------
 * Function to setup the LIS3DH for spi communication, and to initialise the device.
 */
void lis3dh_init(spi_host_device_t spi_bus) {

    lis3dh_spi_init(spi_bus);
    // Spin for a few ms - this is to wait for the device to power up
    esp_rom_delay_us(50);

    // Check the ID register
    LisErrorCode status = LISSUCCESS;
    uint8_t id = lisReadRegister(LIS3DH_WHO_AM_I);
    if (id != WHO_AM_I_CONTENTS) {
        printf(lis_strings[2], id);
        status = LISWHOERROR;
    }

    if (status == LISSUCCESS) {
        lisWriteRegister(LIS3DH_CTRL_REG1, 0x27);
        lisWriteRegister(LIS3DH_CTRL_REG2, 0x00); // No HighPass filter
        lisWriteRegister(LIS3DH_CTRL_REG3, 0x00); // No interrupts
        lisWriteRegister(LIS3DH_CTRL_REG4, 0x08); // High resolution
        lisWriteRegister(LIS3DH_CTRL_REG5, 0x00); // defaults
        lisWriteRegister(LIS3DH_CTRL_REG6, 0x00); // defaults
        printf(lis_strings[0]);
    } else {
        printf(lis_strings[1]);
    }
}

/* lisPowerDown()
 * -------------------
 * Turns off all activity on the LIS3DH.
 */
void lisPowerDown(void) {
    lisWriteRegister(LIS3DH_CTRL_REG1, 0x00);
}

/* lisReadRegister()
 * ---------------
 * Reads the contents of a given register of the LIS3DH.
 * Sends a 1 in the MSB to indicating reading.
 * Returns: Contents of the register as a byte (8 Bit registers).
 */
uint8_t lisReadRegister(uint8_t reg_addr) {

    uint8_t tx_buffer[2] = {LIS3DH_SPI_READ | reg_addr, 0};
    uint8_t rx_buffer[2];
    spi_transaction_t transaction = {
        .length = 16,           // Transaction length in bits
        .tx_buffer = tx_buffer, // Pointer to transmit buffer
        .rx_buffer = rx_buffer, // Pointer to receive buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(lis3dh_spi_handle, &transaction));
    return rx_buffer[1];
}

/* lisWriteRegister()
 * ----------------
 * Writes to the imu's registers.
 *
 * addr: The register on the LIS3DH to write to.
 * data: 8 bit data to write to the register.
 */
void lisWriteRegister(uint8_t reg_addr, uint8_t data) {

    uint8_t tx_buffer[2] = {LIS3DH_SPI_WRITE & reg_addr, data};
    spi_transaction_t transaction = {
        .length = 16,           // Transaction length in bits
        .tx_buffer = tx_buffer, // Pointer to transmit buffer
        .rx_buffer = NULL,      // Pointer to receive buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(lis3dh_spi_handle, &transaction));
}

/* lisReadAxisData()
 * --------------
 * Function to read the raw data for all 3 axis from the LIS3DH
 */
void lisReadAxisData(int16_t* x, int16_t* y, int16_t* z) {

    uint8_t tx_buffer[7] = {LIS3DH_OUT_X_L | LIS3DH_SPI_MULTI_READ};
    uint8_t rx_buffer[7];
    spi_transaction_t transaction = {
        .length = 8 * LIS3DH_AXIS_DATA_LENGTH, // Transaction length in bits
        .tx_buffer = tx_buffer,                // Pointer to transmit buffer
        .rx_buffer = rx_buffer,                // Pointer to receive buffer
    };

    ESP_ERROR_CHECK(spi_device_transmit(lis3dh_spi_handle, &transaction));

    int16_t rawX = (rx_buffer[2] << 8 | rx_buffer[1]);
    int16_t rawY = (rx_buffer[4] << 8 | rx_buffer[3]);
    int16_t rawZ = (rx_buffer[6] << 8 | rx_buffer[5]);

    // Adjust the values for the Left justification of a 12bit number
    rawX >>= 4;
    rawY >>= 4;
    rawZ >>= 4;
    if (rawX & 0x0800) {
        rawX |= 0xF000;
    }

    if (rawY & 0x0800) {
        rawY |= 0xF000;
    }

    if (rawZ & 0x0800) {
        rawZ |= 0xF000;
    }

    *x = rawX;
    *y = rawY;
    *z = rawZ;
}

/* getPitchAngle()
 * ---------------
 * Calculates pitch (nose up/down) in degrees.
 * Positive = nose up.
 * Assumes:
 *   - X = forward
 *   - Y = left
 *   - Z = up
 */
double getPitchAngle(int16_t x, int16_t y, int16_t z) {
    double accX = x * LIS3DH_SENSITIVITY_2G / 1000.0; // Convert mg → g
    double accY = y * LIS3DH_SENSITIVITY_2G / 1000.0;
    double accZ = z * LIS3DH_SENSITIVITY_2G / 1000.0;

    return atan2(accX, sqrt(pow(accY,2) + pow(accZ,2))) * RAD_TO_DEG;
}

/* getRollAngle()
 * --------------
 * Calculates roll (left/right wing up/down) in degrees.
 * Positive = left wing up.
 * Assumes:
 *   - X = forward
 *   - Y = left
 *   - Z = up
 */
double getRollAngle(int16_t y, int16_t z) {
    double accY = y * LIS3DH_SENSITIVITY_2G / 1000.0; // Convert mg → g
    double accZ = z * LIS3DH_SENSITIVITY_2G / 1000.0;

    return atan2(accY, accZ) * RAD_TO_DEG;
}
