/**
 * @file pid.cpp
 * @brief Robust Cascaded PID Controller Implementation with Anti-Windup & LPF D-Term.
 */

#include "pid.h"
#include <cmath>
#include <algorithm>

PIDController::PIDController()
    : integral_(0.0f),
      prev_error_(0.0f),
      d_filtered_(0.0f),
      last_error_(0.0f),
      p_term_(0.0f),
      i_term_(0.0f),
      d_term_(0.0f),
      first_run_(true) {
    config_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.005f};
}

PIDController::PIDController(const PIDConfig &config)
    : config_(config),
      integral_(0.0f),
      prev_error_(0.0f),
      d_filtered_(0.0f),
      last_error_(0.0f),
      p_term_(0.0f),
      i_term_(0.0f),
      d_term_(0.0f),
      first_run_(true) {
}

void PIDController::setConfig(const PIDConfig &config) {
    config_ = config;
    reset();
}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    d_filtered_ = 0.0f;
    last_error_ = 0.0f;
    p_term_ = 0.0f;
    i_term_ = 0.0f;
    d_term_ = 0.0f;
    first_run_ = true;
}

float PIDController::update(float setpoint, float measurement, float dt) {
    // 1. Guard against invalid arguments, NaN or Inf
    if (std::isnan(setpoint) || std::isnan(measurement) || std::isnan(dt) ||
        std::isinf(setpoint) || std::isinf(measurement) || std::isinf(dt) ||
        dt <= 0.00001f || dt > 0.1f) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    last_error_ = error;

    // 2. Proportional term
    p_term_ = config_.kp * error;

    // 3. Derivative term with low-pass filtering to suppress high-frequency noise
    float raw_derivative = 0.0f;
    if (!first_run_) {
        raw_derivative = (error - prev_error_) / dt;
    } else {
        first_run_ = false;
    }

    // First order low-pass filter: alpha_d = dt / (tau + dt)
    float alpha_d = 1.0f;
    if (config_.d_filter_tau > 0.0001f) {
        alpha_d = dt / (config_.d_filter_tau + dt);
    }
    d_filtered_ = d_filtered_ + alpha_d * (raw_derivative - d_filtered_);
    d_term_ = config_.kd * d_filtered_;

    prev_error_ = error;

    // 4. Tentative output calculation before integral to check for saturation
    float unsaturated_output = p_term_ + (config_.ki * integral_) + d_term_;

    // 5. Anti-windup conditional integration:
    // Only accumulate integral if output is NOT saturating, OR if error is driving
    // the system OUT of saturation
    bool saturating_positive = (unsaturated_output >= config_.max_output);
    bool saturating_negative = (unsaturated_output <= -config_.max_output);

    bool allow_integration = true;
    if (saturating_positive && error > 0.0f) {
        allow_integration = false; // Prevent worsening positive saturation
    } else if (saturating_negative && error < 0.0f) {
        allow_integration = false; // Prevent worsening negative saturation
    }

    if (allow_integration && config_.ki > 0.0f) {
        integral_ += error * dt;

        // Hard clamping on integral accumulator
        if (integral_ > config_.max_integral) {
            integral_ = config_.max_integral;
        } else if (integral_ < -config_.max_integral) {
            integral_ = -config_.max_integral;
        }
    }
    i_term_ = config_.ki * integral_;

    // 6. Total combined output
    float total_output = p_term_ + i_term_ + d_term_;

    // 7. Final output clamping
    if (total_output > config_.max_output) {
        total_output = config_.max_output;
    } else if (total_output < -config_.max_output) {
        total_output = -config_.max_output;
    }

    // Sanity check
    if (std::isnan(total_output) || std::isinf(total_output)) {
        return 0.0f;
    }

    return total_output;
}

AxisController::AxisController(const PIDConfig &angle_cfg, const PIDConfig &rate_cfg)
    : angle_pid_(angle_cfg),
      rate_pid_(rate_cfg) {
}

void AxisController::reset() {
    angle_pid_.reset();
    rate_pid_.reset();
}

float AxisController::updateCascaded(float desired_angle, float current_angle,
                                    float current_rate, float dt) {
    // Outer loop: Angle error -> Desired angular rate
    float target_rate = angle_pid_.update(desired_angle, current_angle, dt);

    // Inner loop: Rate error -> Output torque demand
    return rate_pid_.update(target_rate, current_rate, dt);
}

float AxisController::updateRateOnly(float desired_rate, float current_rate, float dt) {
    angle_pid_.reset();
    return rate_pid_.update(desired_rate, current_rate, dt);
}
