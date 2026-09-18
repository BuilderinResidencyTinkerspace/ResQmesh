/**
 * @file packet.h
 * @brief Common ESP-NOW Control Packet definition matching Drone Firmware.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#pragma pack(push, 1)
struct ControlPacket {
    uint16_t throttle;  ///< Stick throttle: 0 to 1000
    int16_t  roll;      ///< Stick roll:     -500 to +500
    int16_t  pitch;     ///< Stick pitch:    -500 to +500
    int16_t  yaw;       ///< Stick yaw:      -500 to +500
    uint8_t  arm;       ///< 0 = Disarm, 1 = Arm, 2 = Emergency Stop
    uint8_t  mode;      ///< 0 = Angle, 1 = Rate, 2 = Calibrate
    uint32_t sequence;  ///< Sequence number
    uint16_t crc;       ///< CRC-16-CCITT
};
#pragma pack(pop)

static inline uint16_t computePacketCRC(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}
