/**
 * @file pid.h
 * @brief Robust Cascaded PID Controller with Anti-Windup and Low-Pass D-Filter.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief PID Gain and Constraint Configuration Parameters.
 */
struct PIDConfig {
    float kp;               ///< Proportional gain
    float ki;               ///< Integral gain
    float kd;               ///< Derivative gain
    float max_integral;     ///< Anti-windup clamping boundary [units]
    float max_output;       ///< Hard output saturation limit [units]
    float d_filter_tau;     ///< First-order low-pass filter time constant for derivative [s]
};

class PIDController {
public:
    PIDController();
    PIDController(const PIDConfig &config);

    /**
     * @brief Set or update PID gains and limits.
     */
    void setConfig(const PIDConfig &config);
    const PIDConfig& getConfig() const { return config_; }

    /**
     * @brief Reset integral accumulator and previous derivative states.
     */
    void reset();

    /**
     * @brief Execute PID update step.
     * @param setpoint Desired target value.
     * @param measurement Measured process variable.
     * @param dt Time delta since last update in seconds.
     * @return Computed control output clamped to [-max_output, +max_output].
     */
    float update(float setpoint, float measurement, float dt);

    /**
     * @brief Get individual contribution terms (useful for serial telemetry/tuning).
     */
    float getPTerm() const { return p_term_; }
    float getITerm() const { return i_term_; }
    float getDTerm() const { return d_term_; }
    float getLastError() const { return last_error_; }

private:
    PIDConfig config_;

    float integral_;        ///< Accumulated integral error
    float prev_error_;      ///< Error from previous step
    float d_filtered_;      ///< Low-pass filtered derivative value
    float last_error_;      ///< Last computed error
    float p_term_;          ///< Last proportional output
    float i_term_;          ///< Last integral output
    float d_term_;          ///< Last derivative output
    bool first_run_;        ///< Guard against derivative kick on startup
};

/**
 * @brief Cascaded Axis Controller combining Outer Angle Loop and Inner Rate Loop.
 */
class AxisController {
public:
    AxisController(const PIDConfig &angle_cfg, const PIDConfig &rate_cfg);

    void reset();

    /**
     * @brief Run cascaded update: Desired Angle -> Angle PID -> Desired Rate -> Rate PID -> Output.
     * @param desired_angle Pilot angle command [deg]
     * @param current_angle Estimated body angle [deg]
     * @param current_rate  Measured gyro angular rate [deg/s]
     * @param dt            Loop time delta [s]
     * @return Motor mixer torque demand
     */
    float updateCascaded(float desired_angle, float current_angle, float current_rate, float dt);

    /**
     * @brief Run inner rate loop only (e.g. for Acro/Rate mode or Yaw axis).
     * @param desired_rate Desired angular velocity [deg/s]
     * @param current_rate Measured gyro angular velocity [deg/s]
     * @param dt           Loop time delta [s]
     * @return Motor mixer torque demand
     */
    float updateRateOnly(float desired_rate, float current_rate, float dt);

    PIDController& getAnglePID() { return angle_pid_; }
    PIDController& getRatePID() { return rate_pid_; }

private:
    PIDController angle_pid_;
    PIDController rate_pid_;
};
