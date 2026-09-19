/**
 * @file swarm.h
 * @brief ResQmesh ESP-NOW Swarm Communication Protocol & Peer Mesh Subsystem.
 *
 * Implements low-latency (<5ms) multi-agent packet framing, node addressing,
 * decentralized peer state tracking, and swarm-wide coordinated flight commands.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <cstring>
#include "config.h"

// ResQmesh Magic Protocol Identifier ("RQ")
#define SWARM_MAGIC_0 0x52 // 'R'
#define SWARM_MAGIC_1 0x51 // 'Q'

// Swarm Node Addressing
#define SWARM_NODE_BROADCAST 0xFF // Sent to all drones in the swarm
#define SWARM_NODE_GCS       0x00 // Ground Control Station ID

// Swarm Message Types
enum SwarmMsgType : uint8_t {
    SWARM_MSG_CONTROL    = 0x01, ///< Targeted or broadcast pilot stick commands
    SWARM_MSG_HEARTBEAT  = 0x02, ///< Periodic peer state, attitude & battery broadcast
    SWARM_MSG_COMMAND    = 0x03, ///< Coordinated swarm mission commands (arm/disarm/kill)
    SWARM_MSG_TELEMETRY  = 0x04  ///< Detailed node performance metrics
};

// Swarm Node Operational Roles
enum SwarmRole : uint8_t {
    SWARM_ROLE_STANDALONE = 0, ///< Independent flight node
    SWARM_ROLE_LEADER     = 1, ///< Formation leader / swarm master
    SWARM_ROLE_FOLLOWER   = 2, ///< Formation follower node
    SWARM_ROLE_RELAY      = 3  ///< Airborne communication hop repeater
};

// Swarm Coordinated Mission Commands
enum SwarmCommandId : uint8_t {
    CMD_SWARM_NONE           = 0x00,
    CMD_SWARM_ARM_ALL        = 0x01, ///< Synchronized arming for swarm launch
    CMD_SWARM_DISARM_ALL     = 0x02, ///< Swarm-wide disarm
    CMD_SWARM_EMERGENCY_STOP = 0x03, ///< Instant kill-switch for all or targeted nodes
    CMD_SWARM_CALIBRATE_ALL  = 0x04, ///< Synchronized IMU calibration (disarmed only)
    CMD_SWARM_SET_LEADER     = 0x05, ///< Designate new formation leader
    CMD_SWARM_SET_FORMATION  = 0x06  ///< Switch formation pattern (Line, V, Grid, Circle)
};

#pragma pack(push, 1)

/**
 * @brief Standard Swarm Protocol Header.
 */
struct SwarmHeader {
    uint8_t magic[2];   ///< Fixed magic bytes: {'R', 'Q'}
    uint8_t msg_type;   ///< SwarmMsgType
    uint8_t target_id;  ///< Target Node ID (1..254 or SWARM_NODE_BROADCAST)
    uint8_t sender_id;  ///< Sender Node ID (0x00 for GCS)
    uint32_t sequence;  ///< Rolling sequence counter
};

/**
 * @brief Directed / Broadcast Swarm Control Packet.
 */
struct SwarmControlPacket {
    SwarmHeader header;
    uint16_t throttle;  ///< Stick throttle: 0 to 1000
    int16_t  roll;      ///< Stick roll:     -500 to +500
    int16_t  pitch;     ///< Stick pitch:    -500 to +500
    int16_t  yaw;       ///< Stick yaw:      -500 to +500
    uint8_t  arm;       ///< 0 = Disarm, 1 = Arm, 2 = Emergency Stop
    uint8_t  mode;      ///< 0 = Angle, 1 = Rate, 2 = Calib
    uint16_t crc;       ///< CRC-16-CCITT
};

/**
 * @brief 10 Hz Peer State & Health Broadcast Heartbeat.
 */
struct SwarmHeartbeatPacket {
    SwarmHeader header;
    uint8_t  role;              ///< SwarmRole
    uint8_t  state;             ///< DroneState enum
    uint16_t battery_mv;        ///< Battery voltage in millivolts (e.g. 3850)
    uint8_t  battery_pct;       ///< Battery percentage (0-100%)
    int16_t  roll_deg_x10;      ///< Current roll angle * 10 (e.g. 25.4 deg = 254)
    int16_t  pitch_deg_x10;     ///< Current pitch angle * 10
    int16_t  yaw_rate_dps_x10;  ///< Current yaw rate * 10
    uint8_t  armed;             ///< 1 if motors armed, 0 otherwise
    uint32_t uptime_s;          ///< Node uptime in seconds
    uint16_t crc;               ///< CRC-16-CCITT
};

/**
 * @brief High-Level Swarm Mission Command Packet.
 */
struct SwarmCommandPacket {
    SwarmHeader header;
    uint8_t  cmd_id;            ///< SwarmCommandId
    uint8_t  payload[8];        ///< Command arguments (e.g. formation type, leader ID)
    uint16_t crc;               ///< CRC-16-CCITT
};

/**
 * @brief Extended Diagnostic Telemetry Packet.
 */
struct SwarmTelemetryPacket {
    SwarmHeader header;
    uint16_t motor_duty[4];     ///< PWM duties (0..1023) sent to M1..M4
    uint16_t loop_freq_hz;      ///< Actual flight loop frequency (e.g. 500)
    uint16_t loop_time_us;      ///< Total loop execution latency in microseconds
    uint8_t  packet_loss_pct;   ///< Estimated packet loss percentage
    uint16_t crc;               ///< CRC-16-CCITT
};

#pragma pack(pop)

/**
 * @brief Entry in the local Swarm Peer Mesh Table tracking neighbor drones.
 */
struct SwarmPeer {
    uint8_t  node_id;
    uint8_t  role;
    uint8_t  state;
    uint16_t battery_mv;
    uint8_t  battery_pct;
    float    roll_deg;
    float    pitch_deg;
    float    yaw_rate_dps;
    bool     armed;
    uint32_t uptime_s;
    int64_t  last_seen_us;
    uint32_t packets_received;
    bool     active;
};

/**
 * @brief In-memory decentralized peer table tracking up to SWARM_MAX_PEERS.
 */
class SwarmPeerTable {
public:
    SwarmPeerTable() : peer_count_(0) {
        memset(peers_, 0, sizeof(peers_));
    }

    /**
     * @brief Update or insert peer entry from received heartbeat.
     */
    void updatePeer(const SwarmHeartbeatPacket &hb, int64_t now_us) {
        uint8_t id = hb.header.sender_id;
        if (id == 0 || id == SWARM_NODE_BROADCAST) return;

        // Look for existing peer
        for (size_t i = 0; i < SWARM_MAX_PEERS; i++) {
            if (peers_[i].active && peers_[i].node_id == id) {
                populatePeer(peers_[i], hb, now_us);
                return;
            }
        }

        // Find empty slot for new peer
        for (size_t i = 0; i < SWARM_MAX_PEERS; i++) {
            if (!peers_[i].active) {
                peers_[i].node_id = id;
                peers_[i].packets_received = 0;
                populatePeer(peers_[i], hb, now_us);
                peer_count_++;
                return;
            }
        }

        // Table full: replace oldest peer
        size_t oldest_idx = 0;
        int64_t oldest_time = peers_[0].last_seen_us;
        for (size_t i = 1; i < SWARM_MAX_PEERS; i++) {
            if (peers_[i].last_seen_us < oldest_time) {
                oldest_time = peers_[i].last_seen_us;
                oldest_idx = i;
            }
        }
        peers_[oldest_idx].node_id = id;
        peers_[oldest_idx].packets_received = 0;
        populatePeer(peers_[oldest_idx], hb, now_us);
    }

    /**
     * @brief Purge or deactivate stale peers that have missed heartbeats.
     */
    void cleanupStalePeers(int64_t now_us, int64_t timeout_us) {
        for (size_t i = 0; i < SWARM_MAX_PEERS; i++) {
            if (peers_[i].active) {
                if ((now_us - peers_[i].last_seen_us) > timeout_us) {
                    peers_[i].active = false;
                    if (peer_count_ > 0) peer_count_--;
                }
            }
        }
    }

    size_t getActivePeerCount() const {
        size_t count = 0;
        for (size_t i = 0; i < SWARM_MAX_PEERS; i++) {
            if (peers_[i].active) count++;
        }
        return count;
    }

    const SwarmPeer* getPeer(size_t index) const {
        if (index < SWARM_MAX_PEERS && peers_[index].active) {
            return &peers_[index];
        }
        return nullptr;
    }

    const SwarmPeer* findPeer(uint8_t node_id) const {
        for (size_t i = 0; i < SWARM_MAX_PEERS; i++) {
            if (peers_[i].active && peers_[i].node_id == node_id) {
                return &peers_[i];
            }
        }
        return nullptr;
    }

    void reset() {
        memset(peers_, 0, sizeof(peers_));
        peer_count_ = 0;
    }

private:
    void populatePeer(SwarmPeer &p, const SwarmHeartbeatPacket &hb, int64_t now_us) {
        p.role              = hb.role;
        p.state             = hb.state;
        p.battery_mv        = hb.battery_mv;
        p.battery_pct       = hb.battery_pct;
        p.roll_deg          = (float)hb.roll_deg_x10 / 10.0f;
        p.pitch_deg         = (float)hb.pitch_deg_x10 / 10.0f;
        p.yaw_rate_dps      = (float)hb.yaw_rate_dps_x10 / 10.0f;
        p.armed             = (hb.armed == 1);
        p.uptime_s          = hb.uptime_s;
        p.last_seen_us      = now_us;
        p.packets_received++;
        p.active            = true;
    }

    SwarmPeer peers_[SWARM_MAX_PEERS];
    size_t peer_count_;
};
