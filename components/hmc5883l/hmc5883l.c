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

i2c_master_dev_handle_t magnoI2CHandle;

void i2c_add_device(i2c_master_bus_handle_t bus) {

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, // Address bit length
        .device_address = 0x58,                // Slave address
        .scl_speed_hz = 100000,                // 100kHz
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &magnoI2CHandle));
}

void magnetometer_init(i2c_master_bus_handle_t bus) {
    i2c_add_device(bus);
}

void magnetometer_read_register(uint8_t address) {
}

void magnetometer_write_register(uint8_t address, uint8_t data) {
}
