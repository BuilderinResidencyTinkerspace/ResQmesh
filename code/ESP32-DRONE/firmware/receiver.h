/**
 * @file receiver.h
 * @brief Wireless Flight Control Receiver Subsystem.
 * Supports Wi-Fi SoftAP + WebSocket (Smartphone Touch Controller) and ESP-NOW.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "config.h"

// Flight modes commanded by transmitter
enum FlightMode : uint8_t {
    MODE_ANGLE = 0,     ///< Self-leveling stabilized mode (outer angle + inner rate loop)
    MODE_RATE  = 1,     ///< Acro / Rate mode (inner rate loop only)
    MODE_CALIB = 2      ///< Trigger sensor calibration (only allowed while disarmed)
};

/**
 * @brief 18-byte packed control packet transmitted over ESP-NOW.
 */
#pragma pack(push, 1)
struct ControlPacket {
    uint16_t throttle;  ///< Stick throttle: 0 to 1000
    int16_t  roll;      ///< Stick roll:     -500 to +500
    int16_t  pitch;     ///< Stick pitch:    -500 to +500
    int16_t  yaw;       ///< Stick yaw:      -500 to +500
    uint8_t  arm;       ///< 0 = Disarm, 1 = Arm, 2 = Emergency Stop
    uint8_t  mode;      ///< FlightMode enum value
    uint32_t sequence;  ///< Packet sequence counter (tracks lost packets)
    uint16_t crc;       ///< CRC-16-CCITT packet checksum
};
#pragma pack(pop)

/**
 * @brief Processed normalized control command used by flight loop.
 */
struct ReceiverData {
    float throttle;     ///< Normalized throttle [0.0 to 1000.0]
    float roll_angle;   ///< Desired roll angle [deg] (-STICK_ANGLE_MAX_DEG to +STICK_ANGLE_MAX_DEG)
    float pitch_angle;  ///< Desired pitch angle [deg] (-STICK_ANGLE_MAX_DEG to +STICK_ANGLE_MAX_DEG)
    float yaw_rate;     ///< Desired yaw rate [deg/s] (-STICK_YAW_RATE_MAX_DPS to +STICK_YAW_RATE_MAX_DPS)
    bool  arm_command;  ///< True if transmitter requests arming
    bool  emergency_stop;///< True if transmitter requests instant kill
    FlightMode mode;    ///< Selected flight mode
    uint32_t sequence;  ///< Sequence number of last packet
    uint32_t packets_received;
    uint32_t packets_lost;
    bool  is_connected; ///< True if actively receiving packets within timeout window
};

class Receiver {
public:
    Receiver();
    ~Receiver();

    /**
     * @brief Initialize receiver subsystem (Wi-Fi SoftAP or ESP-NOW).
     * @return true if initialized successfully.
     */
    bool init();

    /**
     * @brief Update receiver status, check timeout, and retrieve latest command.
     * Non-blocking, called from 500 Hz control loop.
     * @param[out] data Target ReceiverData structure.
     * @param now_us Current system timestamp in microseconds.
     * @return true if receiver is healthy and data is fresh.
     */
    bool update(ReceiverData &data, int64_t now_us);

    /**
     * @brief Static callback for ESP-NOW packet reception.
     */
    static void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len);

    /**
     * @brief Store latest drone telemetry to stream back to phone over WebSocket.
     */
    void setTelemetry(float batt_v, float pitch, float roll, const char *state_str, float loop_hz);

    /**
     * @brief Process incoming WebSocket frame from smartphone touch controller.
     */
    void processWebSocketFrame(const char *json_str, size_t len, char *resp_buf, size_t max_resp_len);

    /**
     * @brief Inject simulated control packet (for unit tests / simulation).
     */
    void injectPacket(const ControlPacket &packet, int64_t now_us);

    /**
     * @brief Calculate CRC-16-CCITT for packet validation.
     */
    static uint16_t computeCRC(const uint8_t *data, size_t length);

    bool isConnected() const { return is_connected_; }

    static Receiver *getInstance() { return instance_; }

private:
    void processRawPacket(const ControlPacket &pkt, int64_t now_us);

    ControlPacket latest_packet_;
    int64_t last_packet_time_us_;
    uint32_t last_sequence_;
    uint32_t packets_received_;
    uint32_t packets_lost_;
    bool is_connected_;
    bool is_first_packet_;

    // Telemetry cache for web clients
    float telem_batt_v_;
    float telem_pitch_;
    float telem_roll_;
    char telem_state_[16];
    float telem_loop_hz_;

    static Receiver *instance_;
};
