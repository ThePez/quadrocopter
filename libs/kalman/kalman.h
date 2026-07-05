/*
 *****************************************************************************
 * File: kalman.h
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#ifndef KALMAN_H
#define KALMAN_H

#include "imu.h"

void kalman_update(struct imu_packet_t* sample);
void kalman_get(struct imu_packet_t* out);

#endif
