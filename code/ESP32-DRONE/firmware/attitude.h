/**
 * @file attitude.h
 * @brief Complementary Filter Attitude Estimator for Quadcopter Flight Control.
 *
 * Coordinate conventions:
 * - Frame: Forward-Right-Down (FRD) body frame.
 * - Roll (phi):   Rotation around X-axis. Positive = Right wing down.
 * - Pitch (theta): Rotation around Y-axis. Positive = Nose pitched UP.
 * - Yaw rate (psi_dot): Angular velocity around Z-axis. Positive = Clockwise / Yaw right.
 *
 * Note on Yaw: A 6-axis IMU (accel + gyro) can only estimate absolute Roll and Pitch
 * relative to the gravity vector. Yaw angle cannot be determined drift-free without
 * a magnetometer or external reference; therefore, the yaw axis uses direct gyro rate control.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "imu.h"
#include "config.h"

class AttitudeEstimator {
public:
    AttitudeEstimator(float alpha = ATTITUDE_FILTER_ALPHA);

    /**
     * @brief Reset filter states to zero (or level).
     */
    void reset();

    /**
     * @brief Update the attitude estimate using IMU accelerometer and gyroscope data.
     * @param imu_data Validated physical IMU data.
     * @param dt Time delta since previous update in seconds (typically 0.002s at 500Hz).
     */
    void update(const IMUData &imu_data, float dt);

    /**
     * @brief Estimated Roll angle in degrees.
     */
    float getRoll() const { return roll_deg_; }

    /**
     * @brief Estimated Pitch angle in degrees.
     */
    float getPitch() const { return pitch_deg_; }

    /**
     * @brief Unfiltered or smoothed Yaw Rate in degrees per second.
     */
    float getYawRate() const { return yaw_rate_dps_; }

    /**
     * @brief Gyro Roll Rate in degrees per second (for rate PID).
     */
    float getRollRate() const { return roll_rate_dps_; }

    /**
     * @brief Gyro Pitch Rate in degrees per second (for rate PID).
     */
    float getPitchRate() const { return pitch_rate_dps_; }

    /**
     * @brief Set complementary filter weight alpha (0.0 to 1.0).
     */
    void setAlpha(float alpha) { alpha_ = alpha; }

    /**
     * @brief Get current filter alpha coefficient.
     */
    float getAlpha() const { return alpha_; }

private:
    float alpha_;           ///< Complementary filter coefficient
    float roll_deg_;        ///< Estimated roll angle [deg]
    float pitch_deg_;       ///< Estimated pitch angle [deg]
    float roll_rate_dps_;   ///< Current roll rate [deg/s]
    float pitch_rate_dps_;  ///< Current pitch rate [deg/s]
    float yaw_rate_dps_;    ///< Current yaw rate [deg/s]
    bool initialized_;      ///< True after first valid acceleration sample
};
