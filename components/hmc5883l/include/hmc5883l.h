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

void i2c_add_device(i2c_master_bus_handle_t bus);
void magnetometer_init(i2c_master_bus_handle_t bus);

#endif
