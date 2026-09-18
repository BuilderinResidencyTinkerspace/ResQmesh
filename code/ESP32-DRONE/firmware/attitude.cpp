/**
 * @file attitude.cpp
 * @brief Complementary Filter Attitude Estimator Implementation.
 */

#include "attitude.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

AttitudeEstimator::AttitudeEstimator(float alpha)
    : alpha_(alpha),
      roll_deg_(0.0f),
      pitch_deg_(0.0f),
      roll_rate_dps_(0.0f),
      pitch_rate_dps_(0.0f),
      yaw_rate_dps_(0.0f),
      initialized_(false) {
}

void AttitudeEstimator::reset() {
    roll_deg_ = 0.0f;
    pitch_deg_ = 0.0f;
    roll_rate_dps_ = 0.0f;
    pitch_rate_dps_ = 0.0f;
    yaw_rate_dps_ = 0.0f;
    initialized_ = false;
}

void AttitudeEstimator::update(const IMUData &imu_data, float dt) {
    if (!imu_data.valid || dt <= 0.0f || dt > 0.1f) {
        return; // Reject bad inputs or unrealistic dt
    }

    // Direct angular rates from calibrated gyroscope
    roll_rate_dps_  = imu_data.gx;
    pitch_rate_dps_ = imu_data.gy;
    yaw_rate_dps_   = imu_data.gz;

    // Calculate total acceleration magnitude squared
    float acc_sq = imu_data.ax * imu_data.ax +
                   imu_data.ay * imu_data.ay +
                   imu_data.az * imu_data.az;

    // Only rely on accelerometer gravity reference if total acceleration is close to 1g
    // (e.g. between 0.75g and 1.25g) to reject centripetal and linear flight accelerations
    bool acc_valid = (acc_sq > (0.75f * 0.75f) && acc_sq < (1.25f * 1.25f));

    float roll_acc = 0.0f;
    float pitch_acc = 0.0f;

    if (acc_valid) {
        // Roll: rotation around X axis (phi = atan2(ay, az))
        roll_acc = atan2f(imu_data.ay, imu_data.az) * RAD_TO_DEG;

        // Pitch: rotation around Y axis (theta = atan2(-ax, sqrt(ay^2 + az^2)))
        float pitch_denom = sqrtf(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az);
        if (pitch_denom > 0.001f) {
            pitch_acc = atan2f(-imu_data.ax, pitch_denom) * RAD_TO_DEG;
        }
    }

    if (!initialized_) {
        // First run initialization: seed directly with gravity vector to eliminate initial transient
        if (acc_valid) {
            roll_deg_ = roll_acc;
            pitch_deg_ = pitch_acc;
            initialized_ = true;
        }
        return;
    }

    // 1. Predict state with gyroscope integration (high frequency)
    float roll_gyro_pred = roll_deg_ + roll_rate_dps_ * dt;
    float pitch_gyro_pred = pitch_deg_ + pitch_rate_dps_ * dt;

    // 2. Fuse with accelerometer gravity correction (low frequency)
    if (acc_valid) {
        roll_deg_ = alpha_ * roll_gyro_pred + (1.0f - alpha_) * roll_acc;
        pitch_deg_ = alpha_ * pitch_gyro_pred + (1.0f - alpha_) * pitch_acc;
    } else {
        // High linear acceleration: trust only gyroscope integration temporarily
        roll_deg_ = roll_gyro_pred;
        pitch_deg_ = pitch_gyro_pred;
    }

    // Safety checks against NaN/Inf corruption
    if (std::isnan(roll_deg_) || std::isinf(roll_deg_)) {
        roll_deg_ = 0.0f;
    }
    if (std::isnan(pitch_deg_) || std::isinf(pitch_deg_)) {
        pitch_deg_ = 0.0f;
    }
}
