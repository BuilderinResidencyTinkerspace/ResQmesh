/**
 * @file config.h
 * @brief Central configuration file for ESP32-S3 Quadcopter Flight Controller.
 *
 * All hardware pins, timing constants, PID gains, filter parameters,
 * and safety limits are defined here to eliminate magic numbers.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// 1. FIRMWARE IDENTITY & REVISION
// =============================================================================
#define FIRMWARE_NAME "ESP32-S3-DRONE"
#define FIRMWARE_VERSION "1.0.0"
#define HARDWARE_TARGET "Seeed Studio XIAO ESP32-S3"

// =============================================================================
// 2. HARDWARE PINOUT MAPPING (XIAO ESP32-S3)
// =============================================================================
// I2C Bus Pins (MPU9250)
#define I2C_PORT_NUM 0     // I2C port 0
#define PIN_I2C_SDA 5      // XIAO D4 (GPIO5)
#define PIN_I2C_SCL 6      // XIAO D5 (GPIO6)
#define I2C_FREQ_HZ 400000 // 400 kHz Fast-Mode
#define I2C_TIMEOUT_MS 20  // I2C bus timeout in milliseconds

// Motor PWM Outputs (AO3400A Low-side MOSFET Gates)
// Quad-X Motor mapping:
// M1: Front-Left  (CW)  -> XIAO D2 (GPIO3)
// M2: Front-Right (CCW) -> XIAO D3 (GPIO4)
// M3: Rear-Right  (CW)  -> XIAO D8 (GPIO7)
// M4: Rear-Left   (CCW) -> XIAO D9 (GPIO8)
#define PIN_MOTOR_FL 4 // Motor 1: Front-Left (GPIO4)
#define PIN_MOTOR_FR 1 // Motor 2: Front-Right (GPIO1)
#define PIN_MOTOR_RR 2 // Motor 3: Rear-Right (GPIO2)
#define PIN_MOTOR_RL 3 // Motor 4: Rear-Left (GPIO3)

// Battery Voltage ADC Sense
#define BATTERY_MONITOR_ENABLED                                                \
  0 // 0 = bypass ADC (fixed healthy 3.85V), 1 = active ADC sensing
#define PIN_BATTERY_ADC 8     // GPIO8 / ADC1_CH7
#define BATTERY_ADC_UNIT 1    // ADC Unit 1
#define BATTERY_ADC_CHANNEL 7 // ADC Channel 7
// NOTE: The solder pads on the back of the XIAO ESP32-S3 (BAT+ / BAT-) provide
// power to the board's 3.3V regulator and charging IC, but are NOT internally
// connected to any ADC pin.
// - Set BATTERY_MONITOR_ENABLED to 0 if powering via the back pads without an
// external resistor divider.
// - Set BATTERY_MONITOR_ENABLED to 1 if you wire an external 100k/100k divider
// from BAT+ to an ADC pin.

// Status Indicator LED
#define PIN_STATUS_LED 21 // XIAO ESP32-S3 onboard LED (active LOW)
#define LED_ACTIVE_LOW 1

// =============================================================================
// 3. IMU SENSOR CONFIGURATION (MPU9250 / MPU6500)
// =============================================================================
#define MPU9250_I2C_ADDR 0x68     // Default AD0 = GND (0x69 if AD0 = VCC)
#define MPU9250_WHO_AM_I_VAL 0x71 // MPU9250 expected WHO_AM_I value
#define MPU6500_WHO_AM_I_VAL 0x70 // MPU6500 expected WHO_AM_I value
#define MPU6050_WHO_AM_I_VAL 0x68 // MPU6050 fallback WHO_AM_I value

// Gyroscope Settings: ±1000 dps full scale range
// Sensitivity: 32.8 LSB / (deg/s)
#define GYRO_FS_SEL 2                       // 0=250, 1=500, 2=1000, 3=2000 dps
#define GYRO_SCALE_DPS (1000.0f / 32768.0f) // 0.030517578 deg/s per LSB

// Accelerometer Settings: ±4g full scale range
// Sensitivity: 8192 LSB / g
#define ACCEL_FS_SEL 1                  // 0=2g, 1=4g, 2=8g, 3=16g
#define ACCEL_SCALE_G (4.0f / 32768.0f) // 0.00012207 g per LSB
#define GRAVITY_MSS 9.80665f            // 1 g in m/s^2

// IMU Axis Mapping / Frame Alignment:
// Default assumes sensor mounted flat, chip dot matching drone frame:
// +X forward, +Y right, +Z down (NED frame)
#define IMU_AXIS_SWAP_XY 0
#define IMU_INVERT_X 0
#define IMU_INVERT_Y 0
#define IMU_INVERT_Z 0

// Calibration Settings
#define IMU_CALIB_SAMPLES 500 // Number of samples for zero-bias calculation
#define IMU_MAX_CALIB_MOTION                                                   \
  0.15f // Max allowable g deviation during calibration

// Bench testing mode: 0 = require physical MPU sensor, 1 = simulated IMU for bare-board desk/swarm testing
#define IMU_BENCH_TEST_MODE 0

// =============================================================================
// 4. ATTITUDE ESTIMATION (COMPLEMENTARY FILTER)
// =============================================================================
// Complementary filter weighting alpha:
// angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
// At 500 Hz (dt = 0.002s), alpha = 0.98 gives time constant tau = 0.098 s
#define ATTITUDE_FILTER_ALPHA 0.980f

// Attitude limits
#define MAX_LEGAL_TILT_DEG                                                     \
  60.0f // Beyond this tilt angle, emergency disarm fires

// =============================================================================
// 5. DETERMINISTIC FLIGHT CONTROL LOOP
// =============================================================================
#define CONTROL_LOOP_FREQ_HZ 500 // 500 Hz target frequency
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_FREQ_HZ) // 2000 µs
#define CONTROL_LOOP_DT (1.0f / (float)CONTROL_LOOP_FREQ_HZ)    // 0.002 seconds
#define LOOP_OVERRUN_TOL_US 200 // Overrun warning threshold: 2200 µs

// FreeRTOS Task Configurations
#define FLIGHT_TASK_CORE 1  // Pinned to Core 1 for zero-jitter execution
#define FLIGHT_TASK_PRIO 24 // Near-highest real-time priority
#define FLIGHT_TASK_STACK_SIZE 8192

#define COMM_TASK_CORE 0 // Core 0 handles wireless & telemetry
#define COMM_TASK_PRIO 5
#define COMM_TASK_STACK_SIZE 4096

#define CLI_TASK_CORE 0
#define CLI_TASK_PRIO 3
#define CLI_TASK_STACK_SIZE 4096

// =============================================================================
// 6. CASCADED PID CONTROLLER GAINS & LIMITS
// =============================================================================
// OUTER LOOP: Angle Controller (Angle Error [deg] -> Desired Angular Rate
// [deg/s])
#define PID_ROLL_ANGLE_KP 4.0f
#define PID_ROLL_ANGLE_KI 0.0f // Outer loop Ki usually 0
#define PID_ROLL_ANGLE_KD 0.0f
#define PID_ROLL_ANGLE_MAX_OUT 200.0f // Max rate demand: ±200 deg/s

#define PID_PITCH_ANGLE_KP 4.0f
#define PID_PITCH_ANGLE_KI 0.0f
#define PID_PITCH_ANGLE_KD 0.0f
#define PID_PITCH_ANGLE_MAX_OUT 200.0f // Max rate demand: ±200 deg/s

// INNER LOOP: Angular Rate Controller (Rate Error [deg/s] -> Mixer Torque
// Output)
#define PID_ROLL_RATE_KP 0.65f
#define PID_ROLL_RATE_KI 0.45f
#define PID_ROLL_RATE_KD 0.025f
#define PID_ROLL_RATE_INT_LIMIT 150.0f // Integral windup limit
#define PID_ROLL_RATE_MAX_OUT 400.0f   // Max rate output limit

#define PID_PITCH_RATE_KP 0.65f
#define PID_PITCH_RATE_KI 0.45f
#define PID_PITCH_RATE_KD 0.025f
#define PID_PITCH_RATE_INT_LIMIT 150.0f
#define PID_PITCH_RATE_MAX_OUT 400.0f

#define PID_YAW_RATE_KP 1.20f // Yaw rate only (no outer angle loop)
#define PID_YAW_RATE_KI 0.80f
#define PID_YAW_RATE_KD 0.00f
#define PID_YAW_RATE_INT_LIMIT 100.0f
#define PID_YAW_RATE_MAX_OUT 300.0f

// Rate loop derivative low-pass filter cutoff (reduces motor buzz/heat from
// gyro noise)
#define PID_D_TERM_FILTER_TAU 0.005f // 5 ms time constant (~31.8 Hz cutoff)

// Integrator reset threshold (throttle % below which I-term is kept 0)
#define PID_I_TERM_MIN_THROTTLE 100 // Throttle < 100 (10%) resets integrators

// =============================================================================
// 7. QUAD-X MOTOR MIXER CONFIGURATION
// =============================================================================
// Normalizing boundaries
#define MIXER_OUT_MIN 0.0f
#define MIXER_OUT_MAX 1023.0f // 10-bit PWM range

// Priority anti-saturation:
// If motor saturation occurs, throttle is lowered to preserve roll/pitch
// attitude authority.
#define MIXER_THROTTLE_REDUCTION_ENABLED 1

// =============================================================================
// 8. MOTOR PWM DRIVER (ESP32-S3 LEDC)
// =============================================================================
#define MOTOR_PWM_FREQ_HZ 20000 // 20 kHz ultrasonic frequency (no coil whine)
#define MOTOR_PWM_BITS 10       // 10-bit resolution -> 0 to 1023
#define MOTOR_PWM_MAX_VALUE 1023

#define MOTOR_PWM_OFF 0 // Motors fully off
#define MOTOR_PWM_MIN_SPIN                                                     \
  60 // Minimum duty required to start 720 coreless motor
#define MOTOR_PWM_IDLE_ARM 50 // Spin-on-arm duty (~5%) to indicate armed state
#define MOTOR_PWM_MAX_FLIGHT                                                   \
  800 // Safe flight duty ceiling (prevents battery brownout)

// Safety switch: Must be set to 1 explicitly to enable serial motor test
// command
#define MOTOR_TEST_ENABLED 1

// =============================================================================
// 9. RECEIVER LINK CONFIGURATION (Wi-Fi SoftAP & ESP-NOW Swarm)
// =============================================================================
// Receiver Mode: 0 = ESP-NOW Swarm, 1 = Wi-Fi SoftAP (Smartphone Touch
// Controller)
#define RECEIVER_MODE_WIFI 0

// Wi-Fi SoftAP Configuration (Used when RECEIVER_MODE_WIFI == 1)
#define WIFI_AP_SSID "ResQmesh-Drone"
#define WIFI_AP_PASSWORD "12345678" // Minimum 8 characters for WPA2-PSK
#define WIFI_AP_CHANNEL 6           // 2.4GHz Wi-Fi channel
#define WIFI_MAX_CLIENTS 2          // Max connected devices

// ESP-NOW Configuration (Used when RECEIVER_MODE_WIFI == 0)
#define ESPNOW_WIFI_CHANNEL 1

// Swarm Mesh Configuration
#define SWARM_DEFAULT_NODE_ID 1 // Default Node ID for this drone (1..254)
#define SWARM_MAX_PEERS 16      // Max tracked neighbor nodes in swarm table
#define SWARM_HEARTBEAT_INTERVAL_MS 100 // 10 Hz heartbeat broadcast
#define SWARM_PEER_TIMEOUT_MS 3000 // Peer expired if no heartbeat in 3000 ms

// Safety Watchdog Timeout: Failsafe automatically triggers and zeroes all motor
// outputs if no control packet is received within this timeout window.
#define RECEIVER_TIMEOUT_MS 300 // 300 ms watchdog (anti-flyaway)
#define STICK_THROTTLE_MIN 0
#define STICK_THROTTLE_MAX 1000
#define STICK_ANGLE_MAX_DEG 30.0f     // Max stick tilt command (±30 degrees)
#define STICK_YAW_RATE_MAX_DPS 200.0f // Max stick yaw rate command (±200 deg/s)
#define ARM_THROTTLE_MAX 100          // Throttle must be < 10% to allow arming

// =============================================================================
// 10. BATTERY MONITOR (1S LIPO)
// =============================================================================
// Voltage divider: R1 (high side) = 100k, R2 (low side) = 100k -> Ratio = 2.0
#define BATTERY_DIVIDER_RATIO 2.00f
#define BATTERY_ADC_SAMPLES 16 // Moving average filter samples

// 1S LiPo Voltage Thresholds (Volts)
#define BATTERY_FULL_VOLTS 4.20f
#define BATTERY_WARN_VOLTS 3.50f // Warning trigger (LED blink fast)
#define BATTERY_CRIT_VOLTS 3.30f // Critical trigger (auto-disarm / landing)
#define BATTERY_EMPTY_VOLTS 3.00f

#ifdef __cplusplus
}
#endif
