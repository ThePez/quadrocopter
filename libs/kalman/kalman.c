/*
 *****************************************************************************
 * File: kalman.c
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#include "kalman.h"

static struct imu_packet_t latest;

// Add the kalman filtering logic here later...

void kalman_update(struct imu_packet_t* sample) {
    latest = *sample;
}

void kalman_get(struct imu_packet_t* out) {
    *out = latest;
}
