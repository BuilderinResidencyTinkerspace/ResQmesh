/**
 * @file motors.h
 * @brief LEDC PWM Motor Controller Abstraction for Brushed Coreless Motors.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

class MotorController {
public:
    MotorController();
    ~MotorController();

    /**
     * @brief Initialize GPIOs with strong pulldowns, setup LEDC timer and channels.
     * All motors guaranteed OFF at the end of initialization.
     * @return true if hardware PWM peripheral initialized successfully.
     */
    bool init();

    /**
     * @brief Transition to armed state. Motors enter idle spin if configured.
     */
    void arm();

    /**
     * @brief Transition to disarmed state. Instantly cuts duty to 0 on all channels.
     */
    void disarm();

    /**
     * @brief Immediate emergency stop. Disarms and zeroes all PWM channels.
     */
    void emergencyStop();

    /**
     * @brief Check arming status.
     */
    bool isArmed() const { return is_armed_; }

    /**
     * @brief Set PWM duty for an individual motor [0.0 to 1023.0].
     * If disarmed, value is forced to 0.
     */
    void setMotor(uint8_t index, float duty);

    /**
     * @brief Set PWM duty for all 4 motors simultaneously.
     */
    void setAll(float duty);

    /**
     * @brief Set individual motor outputs from mixer output structure.
     */
    void applyOutputs(const float outputs[4]);

    /**
     * @brief Set all motor channels to 0 duty immediately.
     */
    void stopAll();

    /**
     * @brief Bench motor test utility (only callable if disarmed AND MOTOR_TEST_ENABLED == 1).
     * @param motor_index 0 to 3
     * @param duty_percent 0 to 20% max for safety
     */
    bool testMotor(uint8_t motor_index, float duty_percent);

    /**
     * @brief Get last commanded duty for telemetry.
     */
    uint32_t getCommandedDuty(uint8_t index) const {
        return (index < 4) ? current_duty_[index] : 0;
    }

private:
    void writeDutyHardware(uint8_t channel, uint32_t duty);

    bool is_armed_;
    uint32_t current_duty_[4];
};
