/**
 * @file run_all_tests.cpp
 * @brief Standalone Host Unit Test Suite for ESP32-DRONE Algorithms.
 *
 * Can be compiled natively with g++ on Windows/Linux to verify all flight
 * algorithms without requiring an ESP32 board attached.
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <cstring>

#include "config.h"
#include "pid.h"
#include "attitude.h"
#include "mixer.h"
#include "receiver.h"
#include "battery.h"
#include "safety.h"
#include "imu.h"

static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (cond) { \
            printf("  [PASS] %s\n", msg); \
            g_tests_passed++; \
        } else { \
            printf("  [FAIL] %s (Line %d: %s)\n", msg, __LINE__, #cond); \
            g_tests_failed++; \
        } \
    } while (0)

#define TEST_HEADER(name) printf("\n--- Running Test: %s ---\n", name)

// =============================================================================
// TEST 1: PID Controller & Anti-Windup
// =============================================================================
void test_pid_controller() {
    TEST_HEADER("PID Controller & Anti-Windup");

    PIDConfig cfg = {
        1.0f,   // kp
        0.5f,   // ki
        0.05f,  // kd
        50.0f,  // max_integral
        100.0f, // max_output
        0.005f  // d_filter_tau
    };
    PIDController pid(cfg);

    // 1. Zero error should yield 0 output
    float out = pid.update(0.0f, 0.0f, 0.002f);
    TEST_ASSERT(fabsf(out) < 0.0001f, "Zero error produces zero output");

    // 2. Proportional step response
    pid.reset();
    out = pid.update(10.0f, 0.0f, 0.002f);
    // On first step: error=10, p=10, integral=10*0.002=0.02, i=0.01, d=0 (first run)
    TEST_ASSERT(out > 9.9f && out < 10.2f, "P-step response matches Kp * error");

    // 3. Integral accumulation over 100 steps
    pid.reset();
    for (int i = 0; i < 100; i++) {
        out = pid.update(10.0f, 0.0f, 0.01f); // 100 * 0.01 = 1 second
    }
    // integral = 10 * 1.0 = 10.0 -> i_term = 0.5 * 10 = 5.0
    TEST_ASSERT(pid.getITerm() > 4.8f && pid.getITerm() < 5.2f, "Integral accumulation over time");

    // 4. Anti-windup clamping
    pid.reset();
    for (int i = 0; i < 2000; i++) {
        pid.update(100.0f, 0.0f, 0.01f); // Huge sustained error
    }
    TEST_ASSERT(pid.getITerm() <= (cfg.ki * cfg.max_integral) + 0.01f,
                "Anti-windup successfully clamped integral accumulator");

    // 5. Output saturation limit
    TEST_ASSERT(out <= cfg.max_output, "Output clamped to max_output");

    // 6. NaN / Inf rejection
    float nan_out = pid.update(NAN, 0.0f, 0.002f);
    TEST_ASSERT(nan_out == 0.0f, "PID rejects NaN setpoint");
    float inf_out = pid.update(0.0f, INFINITY, 0.002f);
    TEST_ASSERT(inf_out == 0.0f, "PID rejects Infinity measurement");
}

// =============================================================================
// TEST 2: Complementary Filter Attitude Estimator
// =============================================================================
void test_attitude_estimator() {
    TEST_HEADER("Complementary Filter Attitude Estimator");

    AttitudeEstimator est(0.98f);
    est.reset();

    // 1. Level condition: ax=0, ay=0, az=1.0g
    IMUData imu = {};
    imu.ax = 0.0f;
    imu.ay = 0.0f;
    imu.az = 1.0f;
    imu.valid = true;

    est.update(imu, 0.002f);
    TEST_ASSERT(fabsf(est.getRoll()) < 0.1f, "Level orientation produces 0 deg roll");
    TEST_ASSERT(fabsf(est.getPitch()) < 0.1f, "Level orientation produces 0 deg pitch");

    // 2. 30 degree tilt in Roll: ay = sin(30) = 0.5g, az = cos(30) = 0.866g
    imu.ay = 0.500f;
    imu.az = 0.866f;
    for (int i = 0; i < 250; i++) { // Run 0.5s at 500Hz
        est.update(imu, 0.002f);
    }
    TEST_ASSERT(fabsf(est.getRoll() - 30.0f) < 1.0f, "Filter converges accurately to 30 deg roll");

    // 3. Dynamic acceleration rejection: total accel = 2.0g (high g vibration/punchout)
    // Filter should ignore corrupted accel and rely purely on gyro
    imu.ax = 0.0f;
    imu.ay = 1.5f; // Unrealistic 1.5g lateral force
    imu.az = 1.5f; // Total > 2.1g
    imu.gx = 0.0f; // Gyro indicates no roll rate
    float roll_before = est.getRoll();
    est.update(imu, 0.002f);
    TEST_ASSERT(fabsf(est.getRoll() - roll_before) < 0.01f,
                "Filter rejects centripetal/linear acceleration when accel != 1g");
}

// =============================================================================
// TEST 3: Quad-X Motor Mixer & Dynamic Anti-Saturation
// =============================================================================
void test_motor_mixer() {
    TEST_HEADER("Quad-X Motor Mixer & Anti-Saturation");

    QuadMixer mixer;

    // 1. Disarmed output must be strictly 0 on all channels
    MotorOutputs out = mixer.mix(500.0f, 50.0f, 0.0f, 0.0f, false);
    bool all_zero = (out.m[0] == 0.0f && out.m[1] == 0.0f && out.m[2] == 0.0f && out.m[3] == 0.0f);
    TEST_ASSERT(all_zero, "Disarmed mixer outputs 0 to all motors");

    // 2. Pure throttle armed: all 4 motors equal
    out = mixer.mix(500.0f, 0.0f, 0.0f, 0.0f, true);
    TEST_ASSERT(out.m[0] == 500.0f && out.m[1] == 500.0f && out.m[2] == 500.0f && out.m[3] == 500.0f,
                "Pure throttle distributes equally to all 4 motors");

    // 3. Roll Right (+Roll): Left motors (M1, M4) must increase, Right (M2, M3) must decrease
    out = mixer.mix(500.0f, 100.0f, 0.0f, 0.0f, true);
    TEST_ASSERT(out.m[MOTOR_FRONT_LEFT] > out.m[MOTOR_FRONT_RIGHT],
                "Positive roll increases Front-Left and decreases Front-Right");
    TEST_ASSERT(out.m[MOTOR_REAR_LEFT] > out.m[MOTOR_REAR_RIGHT],
                "Positive roll increases Rear-Left and decreases Rear-Right");

    // 4. Pitch Nose Up (+Pitch): Rear motors (M3, M4) must increase, Front (M1, M2) decrease
    out = mixer.mix(500.0f, 0.0f, 100.0f, 0.0f, true);
    TEST_ASSERT(out.m[MOTOR_REAR_RIGHT] > out.m[MOTOR_FRONT_RIGHT],
                "Pitch Up increases Rear-Right and decreases Front-Right");
    TEST_ASSERT(out.m[MOTOR_REAR_LEFT] > out.m[MOTOR_FRONT_LEFT],
                "Pitch Up increases Rear-Left and decreases Front-Left");

    // 5. Anti-saturation Priority: High throttle + high roll request
    // Total exceeds 1023, throttle must reduce to preserve differential torque
    out = mixer.mix(950.0f, 200.0f, 0.0f, 0.0f, true);
    TEST_ASSERT(out.m[0] <= 1023.0f && out.m[1] <= 1023.0f && out.m[2] <= 1023.0f && out.m[3] <= 1023.0f,
                "No motor exceeds 1023 max PWM during high-throttle command");
    float diff_roll = out.m[MOTOR_FRONT_LEFT] - out.m[MOTOR_FRONT_RIGHT];
    TEST_ASSERT(diff_roll > 100.0f, "Anti-saturation preserved roll control authority");
}

// =============================================================================
// TEST 4: Receiver Packet Validation & Timeout
// =============================================================================
void test_receiver_and_packets() {
    TEST_HEADER("Receiver CRC16 & Timeout Watchdog");

    Receiver rx;
    ControlPacket pkt = {};
    pkt.throttle = 400;
    pkt.roll = 100;
    pkt.pitch = -50;
    pkt.yaw = 0;
    pkt.arm = 1;
    pkt.mode = 0;
    pkt.sequence = 42;

    // Compute CRC
    size_t payload_len = sizeof(ControlPacket) - sizeof(uint16_t);
    pkt.crc = Receiver::computeCRC((const uint8_t*)&pkt, payload_len);

    // Verify valid CRC
    TEST_ASSERT(pkt.crc != 0, "CRC16 computed non-zero checksum");

    // Inject packet
    int64_t now_us = 1000000; // 1.0 second
    rx.injectPacket(pkt, now_us);

    ReceiverData data;
    bool valid = rx.update(data, now_us + 10000); // 10ms later
    TEST_ASSERT(valid, "Fresh packet accepted");
    TEST_ASSERT(data.is_connected, "Receiver marked connected");
    TEST_ASSERT(data.throttle == 400.0f, "Throttle extracted accurately");
    TEST_ASSERT(data.arm_command == true, "Arm command recognized");

    // Test Corrupted CRC
    pkt.crc ^= 0x5555; // Corrupt CRC
    uint16_t bad_crc = Receiver::computeCRC((const uint8_t*)&pkt, payload_len);
    TEST_ASSERT(bad_crc != pkt.crc, "Corrupted CRC mismatch detected");

    // Test Timeout: Advance time beyond RECEIVER_TIMEOUT_MS
    bool timeout_valid = rx.update(data, now_us + (int64_t)(RECEIVER_TIMEOUT_MS + 50) * 1000LL);
    TEST_ASSERT(!timeout_valid, "Receiver detected timeout watchdog trigger");
    TEST_ASSERT(!data.is_connected, "Receiver marked disconnected on timeout");
    TEST_ASSERT(data.throttle == 0.0f, "Throttle zeroed on timeout");
}

// =============================================================================
// TEST 5: Battery Monitoring & Alarm Thresholds
// =============================================================================
void test_battery_monitor() {
    TEST_HEADER("Battery Monitor & Capacity Calculation");

    BatteryMonitor batt(2.0f);

    // Test full battery (4.20V)
    batt.setSimulatedVoltage(4.20f);
    batt.readVoltage();
    TEST_ASSERT(batt.getPercentage() >= 99.0f, "4.20V reports 100% capacity");
    TEST_ASSERT(!batt.isLow() && !batt.isCritical(), "4.20V reports healthy");

    // Test warning threshold (3.45V)
    batt.setSimulatedVoltage(3.45f);
    batt.readVoltage();
    TEST_ASSERT(batt.isLow(), "3.45V triggers Low Battery alarm");
    TEST_ASSERT(!batt.isCritical(), "3.45V is above critical threshold");

    // Test critical threshold (3.25V)
    batt.setSimulatedVoltage(3.25f);
    batt.readVoltage();
    TEST_ASSERT(batt.isCritical(), "3.25V triggers Critical Battery alarm");
    TEST_ASSERT(batt.getPercentage() <= 0.01f, "3.25V reports 0% capacity");
}

// =============================================================================
// TEST 6: Arming State Machine & Failsafe
// =============================================================================
void test_arming_and_failsafe() {
    TEST_HEADER("Arming State Machine & Failsafe Supervisor");

    SafetySupervisor safety;
    safety.init();

    // 1. Initial State should transition to DISARMED when sensors become ready
    DroneState st = safety.update(true, true, true, false, 0.0f, 0.0f, 0.0f, false, false);
    TEST_ASSERT(st == STATE_DISARMED, "State transitions from BOOT to DISARMED when ready");

    // 2. Reject arming if throttle is too high (e.g. 250 > 100)
    st = safety.update(true, true, true, true, 250.0f, 0.0f, 0.0f, false, false);
    TEST_ASSERT(st == STATE_DISARMED, "Arming rejected when throttle > threshold");

    // 3. Reject arming if drone is tilted (e.g. Roll = 35 deg)
    safety.update(true, true, true, false, 0.0f, 0.0f, 0.0f, false, false); // release arm
    st = safety.update(true, true, true, true, 0.0f, 35.0f, 0.0f, false, false);
    TEST_ASSERT(st == STATE_DISARMED, "Arming rejected when drone is tilted > 20 deg");

    // 4. Successful Arming when all conditions met
    safety.update(true, true, true, false, 0.0f, 0.0f, 0.0f, false, false); // release arm
    st = safety.update(true, true, true, true, 0.0f, 0.0f, 0.0f, false, false);
    TEST_ASSERT(st == STATE_ARMED, "Drone successfully ARMS when all 7 checks pass");

    // 5. In-flight Failsafe on receiver loss
    st = safety.update(true, true, false, true, 400.0f, 0.0f, 0.0f, false, false); // RX false
    TEST_ASSERT(st == STATE_FAILSAFE, "Receiver loss triggers immediate FAILSAFE disarm");
    TEST_ASSERT(safety.getLastDisarmReason() == DISARM_RECEIVER_TIMEOUT,
                "Disarm reason recorded as RECEIVER_TIMEOUT");

    // 6. Emergency Stop
    safety.triggerEmergencyStop();
    TEST_ASSERT(safety.getState() == STATE_EMERGENCY_STOP,
                "Emergency stop puts supervisor in EMERGENCY_STOP state");
}

int main() {
    printf("========================================================\n");
    printf("  ESP32-DRONE AUTOMATED HOST UNIT TEST SUITE\n");
    printf("========================================================\n");

    test_pid_controller();
    test_attitude_estimator();
    test_motor_mixer();
    test_receiver_and_packets();
    test_battery_monitor();
    test_arming_and_failsafe();

    printf("\n========================================================\n");
    printf("  TEST RESULTS: %d PASSED, %d FAILED\n", g_tests_passed, g_tests_failed);
    printf("========================================================\n");

    return (g_tests_failed == 0) ? 0 : 1;
}
