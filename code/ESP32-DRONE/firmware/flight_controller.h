/**
 * @file flight_controller.h
 * @brief 500 Hz Deterministic Flight Controller Orchestrator.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "imu.h"
#include "attitude.h"
#include "pid.h"
#include "mixer.h"
#include "motors.h"
#include "receiver.h"
#include "battery.h"
#include "safety.h"

/**
 * @brief Performance profiling and diagnostic telemetry metrics.
 */
struct LoopStats {
    uint32_t loop_count;
    float current_freq_hz;
    uint32_t loop_time_us;
    uint32_t imu_read_time_us;
    uint32_t compute_time_us;
    uint32_t overruns;
};

class FlightController {
public:
    FlightController();
    ~FlightController();

    /**
     * @brief Initialize all hardware subsystems (IMU, Motors, Receiver, Battery, Safety).
     * @return true if all critical peripherals initialized successfully.
     */
    bool init();

    /**
     * @brief Start the 500 Hz deterministic FreeRTOS flight loop task.
     */
    bool start();

    /**
     * @brief Single-step execution of flight control iteration.
     * Accessible directly for unit-testing and software simulation.
     * @param now_us Timestamp in microseconds
     */
    void runIteration(int64_t now_us);

    /**
     * @brief Trigger zero-bias IMU calibration (only allowed while disarmed).
     */
    bool calibrateSensors();

    /**
     * @brief Arm command from CLI or external interface.
     */
    bool requestArm();

    /**
     * @brief Disarm command.
     */
    void requestDisarm();

    /**
     * @brief Emergency stop.
     */
    void emergencyStop();

    // Accessors for subsystems and telemetry
    IMU& getIMU() { return imu_; }
    AttitudeEstimator& getAttitude() { return attitude_; }
    MotorController& getMotors() { return motors_; }
    Receiver& getReceiver() { return receiver_; }
    BatteryMonitor& getBattery() { return battery_; }
    SafetySupervisor& getSafety() { return safety_; }
    QuadMixer& getMixer() { return mixer_; }
    AxisController& getRollController() { return roll_axis_; }
    AxisController& getPitchController() { return pitch_axis_; }
    PIDController& getYawController() { return yaw_rate_pid_; }
    const LoopStats& getStats() const { return stats_; }

    const IMUData& getLatestIMUData() const { return latest_imu_; }
    const ReceiverData& getLatestReceiverData() const { return latest_rx_; }
    const MotorOutputs& getLatestMotorOutputs() const { return latest_motors_; }

private:
    static void flightLoopTask(void *arg);

    IMU imu_;
    AttitudeEstimator attitude_;
    MotorController motors_;
    Receiver receiver_;
    BatteryMonitor battery_;
    SafetySupervisor safety_;
    QuadMixer mixer_;

    AxisController roll_axis_;
    AxisController pitch_axis_;
    PIDController yaw_rate_pid_;

    IMUData latest_imu_;
    ReceiverData latest_rx_;
    MotorOutputs latest_motors_;

    int64_t last_loop_time_us_;
    LoopStats stats_;

    uint32_t perf_timer_start_us_;
    uint32_t sample_counter_;
    int64_t last_stat_calc_us_;
};
