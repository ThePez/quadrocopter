/*
 ******************************************************************************
 * File: hmc5883l.c
 * Author: Jack Cairns
 * Date: 09-07-2025
 * Brief: Driver for the HMC5883L magnetometer
 * REFERENCE: None
 ******************************************************************************
 */

#include "hmc5883l.h"

#include <math.h>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

i2c_master_dev_handle_t magnoI2CHandle;

void i2c_add_device(i2c_master_bus_handle_t bus) {

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, // Address bit length
        .device_address = 0x1E,                // Slave address
        .scl_speed_hz = 100000,                // 100kHz
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &magnoI2CHandle));
}

void magnetometer_init(i2c_master_bus_handle_t bus) {
    i2c_add_device(bus);

    uint8_t config_a[2] = {HMC5883L_CONFIG_A_REG_ADDR, 0x70}; // 1 Sample Average, 15Hz, No Bias
    uint8_t config_b[2] = {HMC5883L_CONFIG_B_REG_ADDR, 0x20}; // +/- 1.3 Ga (Default)
    uint8_t mode[2] = {HMC5883L_MODE_REG_ADDR, 0x00};         // Continuous measurement mode

    // Write Register A
    ESP_ERROR_CHECK(i2c_master_transmit(magnoI2CHandle, config_a, 2, -1));
    // Write Register B
    ESP_ERROR_CHECK(i2c_master_transmit(magnoI2CHandle, config_b, 2, -1));
    // Write Mode Register
    ESP_ERROR_CHECK(i2c_master_transmit(magnoI2CHandle, mode, 2, -1));

    printf("Compass Setup\r\n");
}

uint8_t magnetometer_read_register(uint8_t address) {
    uint8_t data = 0;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(magnoI2CHandle, &address, 1, &data, 1, 5));
    return data;
}

void magnetometer_write_register(uint8_t address, uint8_t data) {
    uint8_t toSend[2] = {address, data};
    ESP_ERROR_CHECK(i2c_master_transmit(magnoI2CHandle, toSend, 2, 5));
}

void magnetometer_read_axis(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t addr = HMC5883L_OUTPUT_X_MSB_REG_ADDR;
    uint8_t outputBuffer[6];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(magnoI2CHandle, &addr, 1, outputBuffer, 6, 1000));
    *x = ((int16_t) outputBuffer[0] << 8) | outputBuffer[1];
    *z = ((int16_t) outputBuffer[2] << 8) | outputBuffer[3];
    *y = ((int16_t) outputBuffer[4] << 8) | outputBuffer[5];
}

/* getYawAngle()
 * -------------
 * Calculates yaw (heading) in degrees.
 * Uses tilt-compensated magnetometer.
 * Positive = nose right (clockwise from North).
 * Assumes:
 *   - X = forward
 *   - Y = left
 *   - Z = up
 */
double getYawAngle(double pitch, double roll, int16_t x, int16_t y, int16_t z) {
    double radPitch = pitch * DEG_TO_RAD;
    double radRoll = roll * DEG_TO_RAD;

    double magX = x * HMC5883L_SCALE; // Gauss
    double magY = y * HMC5883L_SCALE;
    double magZ = z * HMC5883L_SCALE;

    // Tilt-compensation
    double mxh = magX * cos(radPitch) + magZ * sin(radPitch);
    double myh = magX * sin(radRoll) * sin(radPitch) + magY * cos(radRoll) - magZ * sin(radRoll) * cos(radPitch);

    double yaw = atan2(-myh, mxh) * RAD_TO_DEG;

    // Normalize to [0, 360)
    if (yaw < 0)
        yaw += 360.0;

    return yaw;
}
