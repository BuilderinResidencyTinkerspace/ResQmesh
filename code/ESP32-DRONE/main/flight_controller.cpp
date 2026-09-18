/**
 * @file flight_controller.cpp
 * @brief 500 Hz Deterministic Flight Controller Orchestrator Implementation.
 */

#include "flight_controller.h"
#include <cmath>
#include <cstring>

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char *TAG = "FC";
#else
#include <cstdio>
#define ESP_LOGI(tag, fmt, ...) printf("[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERROR][%s] " fmt "\n", tag, ##__VA_ARGS__)
static const char *TAG = "FC_HOST";
#endif

FlightController::FlightController()
    : attitude_(ATTITUDE_FILTER_ALPHA),
      battery_(BATTERY_DIVIDER_RATIO),
      roll_axis_({PID_ROLL_ANGLE_KP, PID_ROLL_ANGLE_KI, PID_ROLL_ANGLE_KD, 0.0f, PID_ROLL_ANGLE_MAX_OUT, 0.0f},
                 {PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, PID_ROLL_RATE_INT_LIMIT, PID_ROLL_RATE_MAX_OUT, PID_D_TERM_FILTER_TAU}),
      pitch_axis_({PID_PITCH_ANGLE_KP, PID_PITCH_ANGLE_KI, PID_PITCH_ANGLE_KD, 0.0f, PID_PITCH_ANGLE_MAX_OUT, 0.0f},
                  {PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, PID_PITCH_RATE_INT_LIMIT, PID_PITCH_RATE_MAX_OUT, PID_D_TERM_FILTER_TAU}),
      yaw_rate_pid_({PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD, PID_YAW_RATE_INT_LIMIT, PID_YAW_RATE_MAX_OUT, PID_D_TERM_FILTER_TAU}),
      last_loop_time_us_(0),
      perf_timer_start_us_(0),
      sample_counter_(0),
      last_stat_calc_us_(0) {
    memset(&latest_imu_, 0, sizeof(latest_imu_));
    memset(&latest_rx_, 0, sizeof(latest_rx_));
    memset(&latest_motors_, 0, sizeof(latest_motors_));
    memset(&stats_, 0, sizeof(stats_));
}

FlightController::~FlightController() {
    emergencyStop();
}

bool FlightController::init() {
    ESP_LOGI(TAG, "Initializing Flight Controller Subsystems...");

    // 1. Initialize Motors (Guaranteed OFF)
    if (!motors_.init()) {
        ESP_LOGE(TAG, "Motor driver init failed!");
        return false;
    }

    // 2. Initialize Battery Monitor
    battery_.init();

    // 3. Initialize Safety Supervisor
    safety_.init();

    // 4. Initialize Receiver
    if (!receiver_.init()) {
        ESP_LOGW(TAG, "Receiver init returned false (will retry / wait)");
    }

    // 5. Initialize IMU
    if (!imu_.init()) {
        ESP_LOGE(TAG, "IMU hardware init failed!");
        return false;
    }

    // 6. Perform startup sensor zero-bias calibration
    ESP_LOGI(TAG, "Running startup IMU calibration...");
    if (!calibrateSensors()) {
        ESP_LOGW(TAG, "Startup calibration failed! Check sensor mounting and keep drone motionless.");
    }

    ESP_LOGI(TAG, "All flight controller subsystems initialized successfully.");
    return true;
}

bool FlightController::calibrateSensors() {
    if (safety_.isArmed()) {
        ESP_LOGE(TAG, "Cannot calibrate sensors while drone is ARMED!");
        return false;
    }

    bool success = imu_.calibrate(IMU_CALIB_SAMPLES);
    if (success) {
        attitude_.reset();
    }
    return success;
}

bool FlightController::requestArm() {
    return (safety_.getState() == STATE_ARMED || safety_.getState() == STATE_FLIGHT);
}

void FlightController::requestDisarm() {
    safety_.triggerEmergencyStop();
    motors_.disarm();
    roll_axis_.reset();
    pitch_axis_.reset();
    yaw_rate_pid_.reset();
}

void FlightController::emergencyStop() {
    safety_.triggerEmergencyStop();
    motors_.emergencyStop();
    roll_axis_.reset();
    pitch_axis_.reset();
    yaw_rate_pid_.reset();
}

void FlightController::runIteration(int64_t now_us) {
    int64_t start_time = now_us;

    // 1. Calculate dynamic dt
    float dt = CONTROL_LOOP_DT;
    if (last_loop_time_us_ > 0) {
        int64_t elapsed_us = now_us - last_loop_time_us_;
        if (elapsed_us > 500 && elapsed_us < 20000) {
            dt = (float)elapsed_us * 1e-6f;
        }
    }
    last_loop_time_us_ = now_us;

    // 2. Read IMU burst
    int64_t t_imu_start = now_us;
    bool imu_ok = imu_.read(latest_imu_);
#if defined(ESP_PLATFORM)
    stats_.imu_read_time_us = (uint32_t)(esp_timer_get_time() - t_imu_start);
#else
    stats_.imu_read_time_us = 350;
#endif

    // 3. Update Attitude Estimator
    if (imu_ok && latest_imu_.valid) {
        attitude_.update(latest_imu_, dt);
    }

    // 4. Retrieve latest pilot commands from Receiver
    receiver_.update(latest_rx_, now_us);

    // 5. Battery voltage check
    battery_.readVoltage();

    // 6. Safety State Machine
    DroneState state = safety_.update(
        imu_.isHealthy(),
        imu_.isCalibrated(),
        latest_rx_.is_connected,
        latest_rx_.arm_command,
        latest_rx_.throttle,
        attitude_.getRoll(),
        attitude_.getPitch(),
        battery_.isCritical(),
        latest_rx_.emergency_stop
    );

    // 7. Cascaded PID & Motor Mixer Calculation
    int64_t t_calc_start = now_us;

    // Check calibration request while disarmed
    if (latest_rx_.mode == MODE_CALIB && !safety_.isArmed()) {
        calibrateSensors();
    }

    if (safety_.isArmed()) {
        // Anti-windup reset: if throttle is below idle cutoff, zero all integrators
        if (latest_rx_.throttle < (float)PID_I_TERM_MIN_THROTTLE) {
            roll_axis_.reset();
            pitch_axis_.reset();
            yaw_rate_pid_.reset();
        }

        float roll_torque = 0.0f;
        float pitch_torque = 0.0f;
        float yaw_torque = 0.0f;

        if (latest_rx_.mode == MODE_ANGLE) {
            // Angle mode: Cascaded PID (Outer Angle -> Inner Rate)
            roll_torque = roll_axis_.updateCascaded(
                latest_rx_.roll_angle,
                attitude_.getRoll(),
                attitude_.getRollRate(),
                dt
            );
            pitch_torque = pitch_axis_.updateCascaded(
                latest_rx_.pitch_angle,
                attitude_.getPitch(),
                attitude_.getPitchRate(),
                dt
            );
        } else {
            // Acro / Rate mode: Direct Angular Rate control
            roll_torque = roll_axis_.updateRateOnly(
                latest_rx_.roll_angle * (STICK_YAW_RATE_MAX_DPS / STICK_ANGLE_MAX_DEG),
                attitude_.getRollRate(),
                dt
            );
            pitch_torque = pitch_axis_.updateRateOnly(
                latest_rx_.pitch_angle * (STICK_YAW_RATE_MAX_DPS / STICK_ANGLE_MAX_DEG),
                attitude_.getPitchRate(),
                dt
            );
        }

        // Yaw rate control
        yaw_torque = yaw_rate_pid_.update(latest_rx_.yaw_rate, attitude_.getYawRate(), dt);

        // Mix outputs for Quad-X frame with priority anti-saturation
        latest_motors_ = mixer_.mix(latest_rx_.throttle, roll_torque, pitch_torque, yaw_torque, true);

        // Apply PWM to hardware
        motors_.arm();
        motors_.applyOutputs(latest_motors_.m);
    } else {
        // Disarmed or Failsafe: zero integrators and shut motors completely
        roll_axis_.reset();
        pitch_axis_.reset();
        yaw_rate_pid_.reset();
        latest_motors_ = mixer_.mix(0.0f, 0.0f, 0.0f, 0.0f, false);
        motors_.disarm();
    }

#if defined(ESP_PLATFORM)
    stats_.compute_time_us = (uint32_t)(esp_timer_get_time() - t_calc_start);
    stats_.loop_time_us = (uint32_t)(esp_timer_get_time() - start_time);
#else
    stats_.compute_time_us = 45;
    stats_.loop_time_us = 400;
#endif

    // Stream telemetry to receiver for Mobile Web HUD (10 Hz)
    if (sample_counter_ % 50 == 0) {
        receiver_.setTelemetry(
            battery_.getVoltage(),
            attitude_.getPitch(),
            attitude_.getRoll(),
            safety_.getStateString(),
            stats_.current_freq_hz
        );
    }

    // Update frequency statistics every 500 iterations (~1 second)
    sample_counter_++;
    if (sample_counter_ >= 500) {
        if (last_stat_calc_us_ > 0) {
            int64_t diff = now_us - last_stat_calc_us_;
            if (diff > 0) {
                stats_.current_freq_hz = (float)sample_counter_ * 1000000.0f / (float)diff;
            }
        }
        last_stat_calc_us_ = now_us;
        stats_.loop_count += sample_counter_;
        sample_counter_ = 0;
    }
}

void FlightController::flightLoopTask(void *arg) {
    FlightController *fc = reinterpret_cast<FlightController*>(arg);

#if defined(ESP_PLATFORM)
    ESP_LOGI(TAG, "Starting 500 Hz Flight Control Task on Core %d (Priority %d)",
             FLIGHT_TASK_CORE, FLIGHT_TASK_PRIO);

    int64_t next_wake_us = esp_timer_get_time();

    while (true) {
        next_wake_us += CONTROL_LOOP_PERIOD_US;
        int64_t now_us = esp_timer_get_time();

        // Check for loop timing overrun
        if (now_us > next_wake_us + LOOP_OVERRUN_TOL_US) {
            fc->stats_.overruns++;
            next_wake_us = now_us; // Resync
        }

        // Execute deterministic control loop sequence
        fc->runIteration(now_us);

        // Calculate remaining sleep time to maintain exactly 500 Hz
        int64_t now_after = esp_timer_get_time();
        int64_t remaining_us = next_wake_us - now_after;

        if (remaining_us > 1200) {
            // Yield FreeRTOS task if plenty of time remains
            vTaskDelay(pdMS_TO_TICKS(1));
            // Microsecond spin to target wake edge
            while (esp_timer_get_time() < next_wake_us) {
                esp_rom_delay_us(10);
            }
        } else if (remaining_us > 0) {
            esp_rom_delay_us((uint32_t)remaining_us);
        }
    }
#endif
}

bool FlightController::start() {
#if defined(ESP_PLATFORM)
    BaseType_t ret = xTaskCreatePinnedToCore(
        flightLoopTask,
        "flight_loop",
        FLIGHT_TASK_STACK_SIZE,
        this,
        FLIGHT_TASK_PRIO,
        nullptr,
        FLIGHT_TASK_CORE
    );
    return (ret == pdPASS);
#else
    return true;
#endif
}
