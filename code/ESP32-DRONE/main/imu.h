/**
 * @file imu.h
 * @brief MPU9250 / MPU6500 9-Axis IMU Driver with I2C Fast-Mode and Calibration.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "config.h"

/**
 * @brief Calibrated physical unit IMU measurement structure.
 */
struct IMUData {
    float ax;       ///< Accelerometer X [g]
    float ay;       ///< Accelerometer Y [g]
    float az;       ///< Accelerometer Z [g]
    float gx;       ///< Gyroscope X [deg/s]
    float gy;       ///< Gyroscope Y [deg/s]
    float gz;       ///< Gyroscope Z [deg/s]
    float temp;     ///< Die temperature [deg C]
    bool valid;     ///< True if sensor reading succeeded and passed health checks
};

/**
 * @brief Calibration biases and scale offsets.
 */
struct IMUCalibration {
    float ax_offset;
    float ay_offset;
    float az_offset;
    float gx_offset;
    float gy_offset;
    float gz_offset;
    bool is_calibrated;
};

class IMU {
public:
    IMU();
    ~IMU();

    /**
     * @brief Initialize I2C bus and MPU9250 registers.
     * @return true if WHO_AM_I matches and configuration succeeded.
     */
    bool init();

    /**
     * @brief Burst read raw 14 bytes, apply axis transformation and calibration offsets.
     * @param[out] data Output physical measurement structure.
     * @return true if read was successful and data is valid.
     */
    bool read(IMUData &data);

    /**
     * @brief Perform zero-bias sensor calibration while drone is motionless and disarmed.
     * @param sample_count Number of samples to average (default from config.h).
     * @return true if calibration succeeded within noise limits.
     */
    bool calibrate(uint16_t sample_count = IMU_CALIB_SAMPLES);

    /**
     * @brief Check if IMU is calibrated.
     */
    bool isCalibrated() const { return calibration_.is_calibrated; }

    /**
     * @brief Get current calibration offsets.
     */
    const IMUCalibration& getCalibration() const { return calibration_; }

    /**
     * @brief Set calibration offsets manually (e.g. loaded from NVS).
     */
    void setCalibration(const IMUCalibration &calib) { calibration_ = calib; }

    /**
     * @brief Returns sensor health status.
     */
    bool isHealthy() const { return is_healthy_; }

    /**
     * @brief Get consecutive I2C failure count.
     */
    uint32_t getErrorCount() const { return error_count_; }

    /**
     * @brief Software simulation hook: inject synthetic raw data without hardware I2C.
     */
    void setSimulatedData(const IMUData &sim_data);
    void enableSimulation(bool enable) { simulation_mode_ = enable; }
    bool isSimulation() const { return simulation_mode_; }

private:
    bool writeRegister(uint8_t reg, uint8_t val);
    bool readRegisters(uint8_t reg, uint8_t *buffer, size_t length);

    IMUCalibration calibration_;
    bool is_healthy_;
    uint32_t error_count_;
    bool simulation_mode_;
    IMUData sim_data_;
};
