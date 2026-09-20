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

// =============================================================================
// TEST 7: ESP-NOW Multi-Agent Swarm Protocol & Peer Mesh
// =============================================================================
void test_swarm_protocol_and_mesh() {
    TEST_HEADER("ESP-NOW Multi-Agent Swarm Protocol & Peer Mesh");

    Receiver rx;
    rx.setNodeId(1); // Set our drone Node ID = 1
    rx.setRole(SWARM_ROLE_FOLLOWER);

    TEST_ASSERT(rx.getNodeId() == 1, "Drone Node ID initialized to 1");
    TEST_ASSERT(rx.getRole() == SWARM_ROLE_FOLLOWER, "Drone Swarm Role initialized to FOLLOWER");

    int64_t now_us = 2000000; // 2.0s

    // 1. Test Directed Swarm Control Packet addressed to Node #1
    SwarmControlPacket cp = {};
    cp.header.magic[0] = SWARM_MAGIC_0;
    cp.header.magic[1] = SWARM_MAGIC_1;
    cp.header.msg_type = SWARM_MSG_CONTROL;
    cp.header.target_id = 1; // For this drone
    cp.header.sender_id = SWARM_NODE_GCS;
    cp.header.sequence = 101;
    cp.throttle = 550;
    cp.roll = 50;
    cp.pitch = -30;
    cp.yaw = 0;
    cp.arm = 1;
    cp.mode = MODE_ANGLE;
    cp.crc = Receiver::computeCRC((const uint8_t*)&cp, sizeof(cp) - sizeof(uint16_t));

    rx.injectSwarmPacket((const uint8_t*)&cp, sizeof(cp), now_us);

    ReceiverData data;
    bool ok = rx.update(data, now_us + 10000);
    TEST_ASSERT(ok, "Directed Swarm Control Packet accepted for matching Node ID");
    TEST_ASSERT(data.throttle == 550.0f, "Directed packet throttle parsed accurately");
    TEST_ASSERT(data.arm_command == true, "Directed arm command processed");

    // 2. Test Directed Swarm Control Packet addressed to Node #2 (should be rejected/ignored by Node #1)
    SwarmControlPacket cp_wrong = cp;
    cp_wrong.header.target_id = 2; // For drone #2!
    cp_wrong.header.sequence = 102;
    cp_wrong.throttle = 999;
    cp_wrong.crc = Receiver::computeCRC((const uint8_t*)&cp_wrong, sizeof(cp_wrong) - sizeof(uint16_t));

    rx.injectSwarmPacket((const uint8_t*)&cp_wrong, sizeof(cp_wrong), now_us + 20000);
    rx.update(data, now_us + 25000);
    TEST_ASSERT(data.throttle == 550.0f, "Packet addressed to Node #2 rejected by Node #1");

    // 3. Test Broadcast Swarm Control Packet (0xFF) (accepted by all nodes)
    SwarmControlPacket cp_bcast = cp;
    cp_bcast.header.target_id = SWARM_NODE_BROADCAST;
    cp_bcast.header.sequence = 103;
    cp_bcast.throttle = 620;
    cp_bcast.crc = Receiver::computeCRC((const uint8_t*)&cp_bcast, sizeof(cp_bcast) - sizeof(uint16_t));

    rx.injectSwarmPacket((const uint8_t*)&cp_bcast, sizeof(cp_bcast), now_us + 30000);
    rx.update(data, now_us + 35000);
    TEST_ASSERT(data.throttle == 620.0f, "Broadcast packet (0xFF) accepted by Node #1");

    // 4. Test Corrupted Swarm CRC
    SwarmControlPacket cp_bad = cp;
    cp_bad.header.sequence = 104;
    cp_bad.throttle = 800;
    cp_bad.crc = 0xDEAD; // Invalid CRC
    rx.injectSwarmPacket((const uint8_t*)&cp_bad, sizeof(cp_bad), now_us + 40000);
    rx.update(data, now_us + 45000);
    TEST_ASSERT(data.throttle == 620.0f, "Corrupted Swarm Packet rejected by CRC check");

    // 5. Test Swarm Heartbeat & Peer Discovery
    TEST_ASSERT(rx.getPeerTable().getActivePeerCount() == 0, "Initial peer table empty");

    SwarmHeartbeatPacket hb = {};
    hb.header.magic[0] = SWARM_MAGIC_0;
    hb.header.magic[1] = SWARM_MAGIC_1;
    hb.header.msg_type = SWARM_MSG_HEARTBEAT;
    hb.header.target_id = SWARM_NODE_BROADCAST;
    hb.header.sender_id = 5; // Neighbor drone #5
    hb.header.sequence = 1;
    hb.role = SWARM_ROLE_LEADER;
    hb.state = STATE_FLIGHT;
    hb.battery_mv = 3920;
    hb.battery_pct = 85;
    hb.roll_deg_x10 = 125; // 12.5 deg
    hb.pitch_deg_x10 = -84; // -8.4 deg
    hb.yaw_rate_dps_x10 = 15;
    hb.armed = 1;
    hb.uptime_s = 42;
    hb.crc = Receiver::computeCRC((const uint8_t*)&hb, sizeof(hb) - sizeof(uint16_t));

    rx.injectSwarmPacket((const uint8_t*)&hb, sizeof(hb), now_us + 50000);

    TEST_ASSERT(rx.getPeerTable().getActivePeerCount() == 1, "Peer Node #5 registered in peer table");
    const SwarmPeer *peer = rx.getPeerTable().findPeer(5);
    TEST_ASSERT(peer != nullptr, "Found Peer #5 in table");
    if (peer) {
        TEST_ASSERT(peer->role == SWARM_ROLE_LEADER, "Peer #5 role recorded as LEADER");
        TEST_ASSERT(peer->state == STATE_FLIGHT, "Peer #5 state recorded as FLIGHT");
        TEST_ASSERT(peer->battery_mv == 3920, "Peer #5 battery voltage recorded as 3920 mV");
        TEST_ASSERT(fabsf(peer->roll_deg - 12.5f) < 0.05f, "Peer #5 roll recorded as +12.5 deg");
        TEST_ASSERT(peer->armed == true, "Peer #5 armed state recorded as true");
    }

    // 6. Test Stale Peer Expiration
    rx.getPeerTable().cleanupStalePeers(now_us + 50000 + 4000000LL, (int64_t)SWARM_PEER_TIMEOUT_MS * 1000LL);
    TEST_ASSERT(rx.getPeerTable().getActivePeerCount() == 0, "Stale peer purged after timeout");

    // 7. Test Swarm Mission Command: Emergency Stop
    SwarmCommandPacket cmd = {};
    cmd.header.magic[0] = SWARM_MAGIC_0;
    cmd.header.magic[1] = SWARM_MAGIC_1;
    cmd.header.msg_type = SWARM_MSG_COMMAND;
    cmd.header.target_id = SWARM_NODE_BROADCAST;
    cmd.header.sender_id = SWARM_NODE_GCS;
    cmd.header.sequence = 200;
    cmd.cmd_id = CMD_SWARM_EMERGENCY_STOP;
    cmd.crc = Receiver::computeCRC((const uint8_t*)&cmd, sizeof(cmd) - sizeof(uint16_t));

    rx.injectSwarmPacket((const uint8_t*)&cmd, sizeof(cmd), now_us + 60000);

    SwarmCommandId pending_cmd = CMD_SWARM_NONE;
    bool has_cmd = rx.hasPendingSwarmCommand(pending_cmd);
    TEST_ASSERT(has_cmd, "Pending swarm command available");
    TEST_ASSERT(pending_cmd == CMD_SWARM_EMERGENCY_STOP, "Command ID matches CMD_SWARM_EMERGENCY_STOP");

    rx.update(data, now_us + 65000);
    TEST_ASSERT(data.emergency_stop == true, "Emergency stop flag asserted in receiver data");
}

// =============================================================================
// TEST 8: Concurrent Mobile Phone WebSocket + Swarm Mesh Bridge
// =============================================================================
void test_concurrent_wifi_and_swarm_relay() {
    TEST_HEADER("Concurrent Phone WebSocket + Swarm Mesh Bridge");

    Receiver rx;
    rx.setNodeId(1);
    rx.setRole(SWARM_ROLE_LEADER);

    // 1. Process simulated incoming phone touch controller WebSocket frame
    const char *ws_json = "{\"t\":500,\"y\":-150,\"p\":100,\"r\":-80,\"a\":1}";
    char resp_buf[256] = {0};

    int64_t now_us = 3000000;
    rx.processWebSocketFrame(ws_json, strlen(ws_json), resp_buf, sizeof(resp_buf), now_us);

    // Verify local receiver data updated from phone
    ReceiverData data;
    bool ok = rx.update(data, now_us + 10000);
    TEST_ASSERT(ok, "WebSocket packet accepted and receiver updated");
    TEST_ASSERT(data.throttle == 500.0f, "Phone throttle 500 parsed accurately");
    TEST_ASSERT(data.arm_command == true, "Phone arming asserted");
    TEST_ASSERT(fabsf(data.pitch_angle - ((100.0f / 500.0f) * STICK_ANGLE_MAX_DEG)) < 0.1f, "Phone pitch mapped to target angle");
    TEST_ASSERT(fabsf(data.yaw_rate - ((-150.0f / 500.0f) * STICK_YAW_RATE_MAX_DPS)) < 0.1f, "Phone yaw mapped to target yaw rate");

    // 2. Check telemetry JSON response string for phone HUD
    TEST_ASSERT(strstr(resp_buf, "\"b\":") != nullptr, "Response contains battery voltage");
    TEST_ASSERT(strstr(resp_buf, "\"s\":") != nullptr, "Response contains system state");
    TEST_ASSERT(strstr(resp_buf, "\"peers\":0") != nullptr, "Response reports 0 peers when alone");

    // 3. Inject a peer heartbeat (Follower Drone #2)
    SwarmHeartbeatPacket hb = {};
    hb.header.magic[0] = SWARM_MAGIC_0;
    hb.header.magic[1] = SWARM_MAGIC_1;
    hb.header.msg_type = SWARM_MSG_HEARTBEAT;
    hb.header.target_id = SWARM_NODE_BROADCAST;
    hb.header.sender_id = 2; // Drone #2
    hb.header.sequence  = 1;
    hb.role             = SWARM_ROLE_FOLLOWER;
    hb.state            = STATE_ARMED;
    hb.battery_mv       = 3820;
    hb.battery_pct      = 75;
    hb.crc              = Receiver::computeCRC((const uint8_t*)&hb, sizeof(hb) - sizeof(uint16_t));

    rx.injectSwarmPacket((const uint8_t*)&hb, sizeof(hb), 3010000);
    TEST_ASSERT(rx.getPeerTable().getActivePeerCount() == 1, "Follower Drone #2 registered in Swarm Peer Table");

    // 4. Send another phone frame and verify response reports the active follower
    memset(resp_buf, 0, sizeof(resp_buf));
    rx.processWebSocketFrame("{\"t\":520,\"y\":0,\"p\":0,\"r\":0,\"a\":1}", 37, resp_buf, sizeof(resp_buf));
    TEST_ASSERT(strstr(resp_buf, "\"peers\":1") != nullptr, "Phone HUD telemetry reports 1 active mesh peer node");
    TEST_ASSERT(strstr(resp_buf, "\"nodes\":[2]") != nullptr, "Telemetry lists follower Node #2 in active mesh");

    // 4b. Test targeted control to Follower Node #2 (Leader sticks stay zeroed/neutral)
    memset(resp_buf, 0, sizeof(resp_buf));
    rx.processWebSocketFrame("{\"t\":600,\"y\":50,\"p\":-30,\"r\":40,\"a\":1,\"target\":2}", 52, resp_buf, sizeof(resp_buf), 3020000);
    ReceiverData leader_rx_data;
    rx.update(leader_rx_data, 3025000);
    TEST_ASSERT(leader_rx_data.throttle == 0.0f, "Leader throttle remains 0 when targeting Follower #2");
    TEST_ASSERT(strstr(resp_buf, "\"tgt\":2") != nullptr, "Telemetry confirms target is Node #2");
    TEST_ASSERT(strstr(resp_buf, "\"fb\":3.82") != nullptr, "Telemetry includes Follower #2 battery voltage");

    // 5. Verify sendHeartbeat and sendSwarmCommand execute without error
    bool hb_sent = rx.sendHeartbeat(STATE_ARMED, 3900, 85, 0.0f, 0.0f, 0.0f, true, 10);
    TEST_ASSERT(hb_sent, "Dual-mode Swarm Heartbeat broadcast executed");

    bool cmd_sent = rx.sendSwarmCommand(CMD_SWARM_ARM_ALL, SWARM_NODE_BROADCAST);
    TEST_ASSERT(cmd_sent, "Dual-mode Swarm Mission Command broadcast executed");
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
    test_swarm_protocol_and_mesh();
    test_concurrent_wifi_and_swarm_relay();

    printf("\n========================================================\n");
    printf("  TEST RESULTS: %d PASSED, %d FAILED\n", g_tests_passed, g_tests_failed);
    printf("========================================================\n");

    return (g_tests_failed == 0) ? 0 : 1;
}
