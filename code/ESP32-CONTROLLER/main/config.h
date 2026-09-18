/**
 * @file config.h
 * @brief Configuration for ESP32 Drone Remote Transmitter.
 */

#pragma once

#include <stdint.h>

#define CONTROLLER_VERSION          "1.0.0"
#define ESPNOW_WIFI_CHANNEL         1
#define TRANSMIT_INTERVAL_MS        20      // 50 Hz transmission rate

// Target Peer MAC Address:
// By default, broadcast to FF:FF:FF:FF:FF:FF so any drone on channel 1 receives packets.
// Replace with the drone's specific MAC address for dedicated pairing.
#define USE_BROADCAST_PEER          1

// Hardware Joystick Pinout (Optional analog gimbals)
#define PIN_STICK_THROTTLE          34      // ADC1_CH6
#define PIN_STICK_YAW               35      // ADC1_CH7
#define PIN_STICK_PITCH             32      // ADC1_CH4
#define PIN_STICK_ROLL              33      // ADC1_CH5

// Toggle Switches and Buttons
#define PIN_SWITCH_ARM              25      // Pull-up switch: LOW = Armed, HIGH = Disarmed
#define PIN_SWITCH_MODE             26      // LOW = Rate mode, HIGH = Angle mode
#define PIN_BUTTON_EMERGENCY        27      // Momentary button: LOW = Emergency Stop
