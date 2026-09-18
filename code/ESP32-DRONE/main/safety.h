/**
 * @file safety.h
 * @brief Arming State Machine, Failsafe Supervisor, and Crash Detection.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

enum DroneState : uint8_t {
    STATE_BOOT = 0,
    STATE_INITIALIZING,
    STATE_DISARMED,
    STATE_ARMED,
    STATE_FLIGHT,
    STATE_FAILSAFE,
    STATE_ERROR,
    STATE_EMERGENCY_STOP
};

enum DisarmReason : uint8_t {
    DISARM_PILOT_COMMAND = 0,
    DISARM_RECEIVER_TIMEOUT,
    DISARM_IMU_FAILURE,
    DISARM_EXCESSIVE_TILT,
    DISARM_CRITICAL_BATTERY,
    DISARM_LOOP_OVERRUN,
    DISARM_EMERGENCY_STOP,
    DISARM_SYSTEM_ERROR
};

class SafetySupervisor {
public:
    SafetySupervisor();

    void init();

    /**
     * @brief Run safety state machine evaluation at 500 Hz.
     * @param imu_healthy IMU hardware communication status
     * @param imu_calibrated True if sensor offsets have been zeroed
     * @param receiver_connected True if receiver packets are arriving within timeout
     * @param pilot_arm_cmd True if transmitter arm switch is engaged
     * @param pilot_throttle Stick throttle [0 to 1000]
     * @param roll_deg Current estimated roll angle [deg]
     * @param pitch_deg Current estimated pitch angle [deg]
     * @param battery_critical True if battery voltage is below critical threshold
     * @param emergency_stop_cmd True if emergency kill button triggered
     * @return Current system DroneState
     */
    DroneState update(bool imu_healthy,
                      bool imu_calibrated,
                      bool receiver_connected,
                      bool pilot_arm_cmd,
                      float pilot_throttle,
                      float roll_deg,
                      float pitch_deg,
                      bool battery_critical,
                      bool emergency_stop_cmd);

    /**
     * @brief Explicitly trigger emergency stop.
     */
    void triggerEmergencyStop();

    /**
     * @brief Clear failsafe/error and return to DISARMED if all sensors are healthy.
     */
    bool resetToDisarmed(bool imu_healthy, bool imu_calibrated, bool receiver_connected, float throttle);

    DroneState getState() const { return current_state_; }
    DisarmReason getLastDisarmReason() const { return last_disarm_reason_; }
    const char* getStateString() const;
    const char* getDisarmReasonString() const;

    bool isArmed() const {
        return (current_state_ == STATE_ARMED || current_state_ == STATE_FLIGHT);
    }

private:
    DroneState current_state_;
    DisarmReason last_disarm_reason_;
    bool prev_pilot_arm_;
};
