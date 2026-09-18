/**
 * @file mixer.cpp
 * @brief Quad-X Motor Mixer Implementation with Aerodynamic Sign Rationale.
 *
 * Motor Geometry and Rotation:
 *
 *               FRONT (Nose)
 *                     ^
 *                     |
 *        M1 (CW)      |      M2 (CCW)
 *           \         |         /
 *            \        |        /
 *             \       |       /
 *     <-------+-------X-------+-------> (Roll Right: +)
 *             /       |       \
 *            /        |        \
 *           /         |         \
 *        M4 (CCW)     |      M3 (CW)
 *                     |
 *                REAR (Tail)
 *
 * Sign derivation:
 * 1. Throttle:
 *    All 4 motors push upward equally (+1.0).
 *
 * 2. Roll (Positive = Right side down / roll right):
 *    To roll right, left motors (M1, M4) must provide more lift (+1.0),
 *    and right motors (M2, M3) must provide less lift (-1.0).
 *
 * 3. Pitch (Positive = Nose pitched UP):
 *    To pitch nose up, front motors (M1, M2) must provide less lift (-1.0),
 *    and rear motors (M3, M4) must provide more lift (+1.0).
 *
 * 4. Yaw (Positive = Clockwise rotation / nose right):
 *    According to Newton's 3rd Law, spinning a propeller produces an opposite reactive
 *    torque on the airframe:
 *    - CW props (M1, M3) impart CCW torque on the frame.
 *    - CCW props (M2, M4) impart CW torque on the frame.
 *    To rotate CW (+Yaw), increase CCW motor thrust (+1.0 on M2, M4)
 *    and decrease CW motor thrust (-1.0 on M1, M3).
 */

#include "mixer.h"
#include <cmath>
#include <algorithm>

QuadMixer::QuadMixer() {
    // Default Standard Quad-X Configuration:
    //                      M1(FL)  M2(FR)  M3(RR)  M4(RL)
    matrix_.throttle[0] = 1.0f; matrix_.throttle[1] = 1.0f; matrix_.throttle[2] = 1.0f; matrix_.throttle[3] = 1.0f;
    matrix_.roll[0]     = 1.0f; matrix_.roll[1]     =-1.0f; matrix_.roll[2]     =-1.0f; matrix_.roll[3]     = 1.0f;
    matrix_.pitch[0]    =-1.0f; matrix_.pitch[1]    =-1.0f; matrix_.pitch[2]    = 1.0f; matrix_.pitch[3]    = 1.0f;
    matrix_.yaw[0]      =-1.0f; matrix_.yaw[1]      = 1.0f; matrix_.yaw[2]      =-1.0f; matrix_.yaw[3]      = 1.0f;
}

void QuadMixer::setMatrix(const MixerMatrix &matrix) {
    matrix_ = matrix;
}

MotorOutputs QuadMixer::mix(float throttle, float roll, float pitch, float yaw, bool armed) {
    MotorOutputs out;
    out.saturated = false;

    // Safety: If disarmed, force all motor commands to 0 immediately
    if (!armed) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            out.m[i] = 0.0f;
        }
        return out;
    }

    // Check for NaN or Inf in inputs
    if (std::isnan(throttle) || std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw) ||
        std::isinf(throttle) || std::isinf(roll) || std::isinf(pitch) || std::isinf(yaw)) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            out.m[i] = 0.0f;
        }
        return out;
    }

    // 1. Calculate unconstrained motor outputs from mixer matrix
    for (int i = 0; i < NUM_MOTORS; i++) {
        out.m[i] = (matrix_.throttle[i] * throttle) +
                   (matrix_.roll[i]     * roll) +
                   (matrix_.pitch[i]    * pitch) +
                   (matrix_.yaw[i]      * yaw);
    }

    // 2. Find maximum and minimum motor outputs
    float max_m = out.m[0];
    float min_m = out.m[0];
    for (int i = 1; i < NUM_MOTORS; i++) {
        if (out.m[i] > max_m) max_m = out.m[i];
        if (out.m[i] < min_m) min_m = out.m[i];
    }

#if MIXER_THROTTLE_REDUCTION_ENABLED
    // 3. Dynamic anti-saturation (priority attitude preservation)
    // If highest motor exceeds maximum, reduce throttle across ALL motors equally.
    // This preserves differential torque (roll/pitch/yaw control authority) without clipping.
    if (max_m > MIXER_OUT_MAX) {
        float excess = max_m - MIXER_OUT_MAX;
        for (int i = 0; i < NUM_MOTORS; i++) {
            out.m[i] -= excess;
        }
        out.saturated = true;
    }

    // If lowest motor drops below minimum, raise all motors equally
    if (min_m < MIXER_OUT_MIN) {
        float deficit = MIXER_OUT_MIN - min_m;
        for (int i = 0; i < NUM_MOTORS; i++) {
            out.m[i] += deficit;
        }
        out.saturated = true;
    }
#endif

    // 4. Hard safety clamping within valid PWM range [0, 1023]
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (out.m[i] < MIXER_OUT_MIN) {
            out.m[i] = MIXER_OUT_MIN;
            out.saturated = true;
        } else if (out.m[i] > MIXER_OUT_MAX) {
            out.m[i] = MIXER_OUT_MAX;
            out.saturated = true;
        }
    }

    return out;
}
