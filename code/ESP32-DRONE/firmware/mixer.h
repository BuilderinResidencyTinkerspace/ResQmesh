/**
 * @file mixer.h
 * @brief Quad-X Motor Mixer with Dynamic Anti-Saturation Normalization.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

enum MotorIndex {
    MOTOR_FRONT_LEFT  = 0,  ///< M1: Front-Left  (CW)
    MOTOR_FRONT_RIGHT = 1,  ///< M2: Front-Right (CCW)
    MOTOR_REAR_RIGHT  = 2,  ///< M3: Rear-Right  (CW)
    MOTOR_REAR_LEFT   = 3,  ///< M4: Rear-Left   (CCW)
    NUM_MOTORS        = 4
};

/**
 * @brief Configurable Quad-X Mixer Matrix.
 * Output[i] = Throttle * t[i] + Roll * r[i] + Pitch * p[i] + Yaw * y[i]
 */
struct MixerMatrix {
    float throttle[NUM_MOTORS];
    float roll[NUM_MOTORS];
    float pitch[NUM_MOTORS];
    float yaw[NUM_MOTORS];
};

struct MotorOutputs {
    float m[NUM_MOTORS];    ///< Normalized output values [0.0 to 1023.0]
    bool saturated;         ///< True if any motor had to be constrained
};

class QuadMixer {
public:
    QuadMixer();

    /**
     * @brief Set custom mixer matrix (for alternate layouts e.g. Props-Out or Quad-+).
     */
    void setMatrix(const MixerMatrix &matrix);
    const MixerMatrix& getMatrix() const { return matrix_; }

    /**
     * @brief Compute motor PWM outputs with priority attitude anti-saturation.
     * @param throttle Base throttle command [0.0 to 1000.0]
     * @param roll     Roll torque correction from PID
     * @param pitch    Pitch torque correction from PID
     * @param yaw      Yaw torque correction from PID
     * @param armed    If false, outputs are forced to 0
     * @return MotorOutputs structure containing 4 motor commands
     */
    MotorOutputs mix(float throttle, float roll, float pitch, float yaw, bool armed);

private:
    MixerMatrix matrix_;
};
