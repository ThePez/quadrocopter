/*
 ******************************************************************************
 * File: hmc5883l.h
 * Author: Jack Cairns
 * Date: 09-07-2025
 * Brief: Driver for the HMC5883L magnetometer
 * REFERENCE: None
 ******************************************************************************
 */

#ifndef HMC5883L_H
#define HMC5883L_H

#include <stdint.h>

#include "driver/i2c_master.h"

#define MAGNO_SLAVE_ADDRESS_WRITE 0x3C
#define MAGNO_SLAVE_ADDRESS_READ 0x3D

#define HMC5883L_CONFIG_A_REG_ADDR 0x00
#define HMC5883L_CONFIG_B_REG_ADDR 0x01
#define HMC5883L_MODE_REG_ADDR 0x02
#define HMC5883L_OUTPUT_X_MSB_REG_ADDR 0x03
#define HMC5883L_OUTPUT_X_LSB_REG_ADDR 0x04
#define HMC5883L_OUTPUT_Z_MSB_REG_ADDR 0x05
#define HMC5883L_OUTPUT_Z_LSB_REG_ADDR 0x06
#define HMC5883L_OUTPUT_Y_MSB_REG_ADDR 0x07
#define HMC5883L_OUTPUT_Y_LSB_REG_ADDR 0x08
#define HMC5883L_STATUS_REG_ADDR 0x09
#define HMC5883L_ID_A_REG_ADDR 0x0A
#define HMC5883L_ID_B_REG_ADDR 0x0B
#define HMC5883L_ID_C_REG_ADDR 0x0C

void i2c_add_device(i2c_master_bus_handle_t bus);
void magnetometer_init(i2c_master_bus_handle_t bus);
uint8_t magnetometer_read_register(uint8_t address);
void magnetometer_write_register(uint8_t address, uint8_t data);
void magnetometer_read_axis(int16_t* x, int16_t* y, int16_t* z);

#endif
