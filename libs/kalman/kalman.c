/*
 *****************************************************************************
 * File: kalman.c
 * Author: Jack Cairns
 * Date: 05-07-2026
 *****************************************************************************
 */

#include "kalman.h"

#include <esp_timer.h>
#include <stdbool.h>

// #define ENABLE

#ifdef ENABLE
// Process noise scales with dt^2 (random-walk angle model); measurement noise
// is how much the fused rotation-vector angle is trusted.
#define KALMAN_PROCESS_NOISE 3.0f
#define KALMAN_MEASUREMENT_NOISE 12.0f

struct kalman_axis_t {
    float angle;
    float uncertainty;
};

static struct kalman_axis_t pitchKalman;
static struct kalman_axis_t rollKalman;
static uint64_t lastUpdateUs;
static bool initialised;
#endif
static struct imu_packet_t latest;

// Single-state (angle only) Kalman filter: predict by integrating the gyro
// rate, correct against the measured (fused) angle. F=1, G=dt, H=1 in the
// general form, simplified out since they're constant.
#ifdef ENABLE
static void kalman_1d(float* angle, float* uncertainty, float rate, float measured_angle,
                      float dt) {
    // Predict
    *angle += dt * rate;
    *uncertainty += dt * dt * KALMAN_PROCESS_NOISE;

    // Correct
    float innovation = measured_angle - *angle;
    float innovation_covariance = *uncertainty + KALMAN_MEASUREMENT_NOISE;
    float gain = *uncertainty / innovation_covariance;

    *angle += gain * innovation;
    *uncertainty *= (1.0f - gain);
}
#endif

void kalman_update(struct imu_packet_t* sample) {
#ifdef ENABLE
    uint64_t now = (uint64_t) esp_timer_get_time();

    if (!initialised) {
        // Seed the estimate from the first sample instead of starting at 0,
        // so the filter doesn't have to "catch up" if the drone isn't level.
        pitchKalman.angle = sample->pitchAngle;
        rollKalman.angle = sample->rollAngle;
        lastUpdateUs = now;
        initialised = true;
    } else {
        float dt = (float) (now - lastUpdateUs) * 1e-6f;
        lastUpdateUs = now;

        kalman_1d(&pitchKalman.angle, &pitchKalman.uncertainty, sample->pitchRate,
                  sample->pitchAngle, dt);
        kalman_1d(&rollKalman.angle, &rollKalman.uncertainty, sample->rollRate, sample->rollAngle,
                  dt);
    }

    latest = *sample;
    latest.pitchAngle = pitchKalman.angle;
    latest.rollAngle = rollKalman.angle;
    // Yaw has no absolute reference to correct against (no magnetometer in the
    // game rotation vector), so it passes through unfiltered.
#else
    latest = *sample;
#endif
}

void kalman_get(struct imu_packet_t* out) {
    *out = latest;
}
