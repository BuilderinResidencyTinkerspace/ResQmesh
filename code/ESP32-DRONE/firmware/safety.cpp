/**
 * @file safety.cpp
 * @brief Arming State Machine and Failsafe Supervisor Implementation.
 */

#include "safety.h"
#include <cmath>

#if defined(ESP_PLATFORM)
#include "esp_log.h"
static const char *TAG = "SAFETY";
#else
#include <cstdio>
#define ESP_LOGI(tag, fmt, ...) printf("[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERROR][%s] " fmt "\n", tag, ##__VA_ARGS__)
static const char *TAG = "SAFETY_HOST";
#endif

SafetySupervisor::SafetySupervisor()
    : current_state_(STATE_BOOT),
      last_disarm_reason_(DISARM_PILOT_COMMAND),
      prev_pilot_arm_(false) {
}

void SafetySupervisor::init() {
    current_state_ = STATE_INITIALIZING;
    prev_pilot_arm_ = false;
}

DroneState SafetySupervisor::update(bool imu_healthy,
                                    bool imu_calibrated,
                                    bool receiver_connected,
                                    bool pilot_arm_cmd,
                                    float pilot_throttle,
                                    float roll_deg,
                                    float pitch_deg,
                                    bool battery_critical,
                                    bool emergency_stop_cmd) {
    // 1. Emergency Stop overrides all other logic unconditionally
    if (emergency_stop_cmd) {
        if (current_state_ != STATE_EMERGENCY_STOP) {
            ESP_LOGW(TAG, "EMERGENCY STOP TRIGGERED!");
            last_disarm_reason_ = DISARM_EMERGENCY_STOP;
            current_state_ = STATE_EMERGENCY_STOP;
        }
        prev_pilot_arm_ = pilot_arm_cmd;
        return current_state_;
    }

    // 2. State Machine Transitions
    switch (current_state_) {
        case STATE_BOOT:
        case STATE_INITIALIZING:
            if (imu_healthy && imu_calibrated) {
                current_state_ = STATE_DISARMED;
                ESP_LOGI(TAG, "Initialization complete. Drone is DISARMED and ready.");
            } else if (!imu_healthy) {
                current_state_ = STATE_ERROR;
                last_disarm_reason_ = DISARM_IMU_FAILURE;
            }
            break;

        case STATE_DISARMED: {
            // Check for ARM command (rising edge required: transmitter must cycle arm switch)
            bool arm_requested = (pilot_arm_cmd && !prev_pilot_arm_);

            if (arm_requested) {
                // Verify all 7 safety pre-arm checks:
                if (!imu_healthy) {
                    ESP_LOGE(TAG, "Arming rejected: IMU unhealthy!");
                } else if (!imu_calibrated) {
                    ESP_LOGE(TAG, "Arming rejected: IMU not calibrated!");
                } else if (!receiver_connected) {
                    ESP_LOGE(TAG, "Arming rejected: Receiver disconnected!");
                } else if (pilot_throttle > (float)ARM_THROTTLE_MAX) {
                    ESP_LOGE(TAG, "Arming rejected: Throttle (%.1f) > threshold (%d)!",
                             pilot_throttle, ARM_THROTTLE_MAX);
                } else if (battery_critical) {
                    ESP_LOGE(TAG, "Arming rejected: Battery voltage critical!");
                } else if (fabsf(roll_deg) > 20.0f || fabsf(pitch_deg) > 20.0f) {
                    ESP_LOGE(TAG, "Arming rejected: Drone tilted (Roll: %.1f, Pitch: %.1f)!",
                             roll_deg, pitch_deg);
                } else {
                    current_state_ = STATE_ARMED;
                    ESP_LOGI(TAG, "Arming checks passed. Drone ARMED!");
                }
            }
            break;
        }

        case STATE_ARMED:
        case STATE_FLIGHT: {
            // Check for pilot disarm request
            if (!pilot_arm_cmd) {
                current_state_ = STATE_DISARMED;
                last_disarm_reason_ = DISARM_PILOT_COMMAND;
                ESP_LOGI(TAG, "Disarmed by pilot command.");
                break;
            }

            // Check failsafe conditions during flight:
            if (!receiver_connected) {
                current_state_ = STATE_FAILSAFE;
                last_disarm_reason_ = DISARM_RECEIVER_TIMEOUT;
                ESP_LOGE(TAG, "FAILSAFE: Receiver connection lost!");
                break;
            }

            if (!imu_healthy) {
                current_state_ = STATE_FAILSAFE;
                last_disarm_reason_ = DISARM_IMU_FAILURE;
                ESP_LOGE(TAG, "FAILSAFE: IMU hardware error!");
                break;
            }

            // Crash / excessive tilt detection (e.g. inverted flight or flip)
            if (fabsf(roll_deg) > MAX_LEGAL_TILT_DEG || fabsf(pitch_deg) > MAX_LEGAL_TILT_DEG) {
                current_state_ = STATE_FAILSAFE;
                last_disarm_reason_ = DISARM_EXCESSIVE_TILT;
                ESP_LOGE(TAG, "FAILSAFE: Excessive tilt detected (R: %.1f, P: %.1f)!",
                         roll_deg, pitch_deg);
                break;
            }

            if (battery_critical) {
                current_state_ = STATE_FAILSAFE;
                last_disarm_reason_ = DISARM_CRITICAL_BATTERY;
                ESP_LOGE(TAG, "FAILSAFE: Battery critical cutoff!");
                break;
            }

            // Transition between ARMED (idle) and FLIGHT (active throttle)
            if (pilot_throttle > (float)MOTOR_PWM_MIN_SPIN) {
                current_state_ = STATE_FLIGHT;
            } else {
                current_state_ = STATE_ARMED;
            }
            break;
        }

        case STATE_FAILSAFE:
        case STATE_ERROR:
        case STATE_EMERGENCY_STOP:
            // Remain in lock-out state until explicitly reset with safe pilot inputs
            break;
    }

    prev_pilot_arm_ = pilot_arm_cmd;
    return current_state_;
}

void SafetySupervisor::triggerEmergencyStop() {
    current_state_ = STATE_EMERGENCY_STOP;
    last_disarm_reason_ = DISARM_EMERGENCY_STOP;
}

bool SafetySupervisor::resetToDisarmed(bool imu_healthy, bool imu_calibrated,
                                       bool receiver_connected, float throttle) {
    if (imu_healthy && imu_calibrated && receiver_connected && throttle < (float)ARM_THROTTLE_MAX) {
        current_state_ = STATE_DISARMED;
        last_disarm_reason_ = DISARM_PILOT_COMMAND;
        prev_pilot_arm_ = true; // Require cycling switch to arm again
        ESP_LOGI(TAG, "Safety system reset to DISARMED.");
        return true;
    }
    return false;
}

const char* SafetySupervisor::getStateString() const {
    switch (current_state_) {
        case STATE_BOOT:           return "BOOT";
        case STATE_INITIALIZING:   return "INITIALIZING";
        case STATE_DISARMED:       return "DISARMED";
        case STATE_ARMED:          return "ARMED";
        case STATE_FLIGHT:         return "FLIGHT";
        case STATE_FAILSAFE:       return "FAILSAFE";
        case STATE_ERROR:          return "ERROR";
        case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
        default:                   return "UNKNOWN";
    }
}

const char* SafetySupervisor::getDisarmReasonString() const {
    switch (last_disarm_reason_) {
        case DISARM_PILOT_COMMAND:     return "PILOT_COMMAND";
        case DISARM_RECEIVER_TIMEOUT:  return "RECEIVER_TIMEOUT";
        case DISARM_IMU_FAILURE:       return "IMU_FAILURE";
        case DISARM_EXCESSIVE_TILT:    return "EXCESSIVE_TILT";
        case DISARM_CRITICAL_BATTERY:  return "CRITICAL_BATTERY";
        case DISARM_LOOP_OVERRUN:      return "LOOP_OVERRUN";
        case DISARM_EMERGENCY_STOP:    return "EMERGENCY_STOP";
        case DISARM_SYSTEM_ERROR:      return "SYSTEM_ERROR";
        default:                       return "NONE";
    }
}
