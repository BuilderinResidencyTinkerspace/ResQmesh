# ESP32 Drone Remote Transmitter Firmware

This project contains the ESP-NOW transmitter firmware for commanding the **ESP32-DRONE** flight controller.

## Features
* **ESP-NOW 2.4 GHz Low-Latency Protocol**: Transmits 18-byte packed control frames at 50 Hz.
* **CRC16-CCITT Verification**: Ensures zero corrupt packets reach the flight controller.
* **Dual Input Modes**:
  1. **Hardware Analog Gimbals / Switches**: Standard RC potentiometers on ADC pins + toggle switches.
  2. **Interactive Serial Keyboard Control**: Full bench testing via USB-UART terminal without requiring physical joysticks.

## Pinout
| Function | GPIO Pin | Notes |
| :--- | :--- | :--- |
| **Throttle Gimbal** | GPIO34 | ADC1_CH6 (0-3.3V) |
| **Yaw Gimbal** | GPIO35 | ADC1_CH7 (0-3.3V) |
| **Pitch Gimbal** | GPIO32 | ADC1_CH4 (0-3.3V) |
| **Roll Gimbal** | GPIO33 | ADC1_CH5 (0-3.3V) |
| **Arm / Disarm Switch** | GPIO25 | SPST toggle to GND (internal pull-up) |
| **Flight Mode Switch** | GPIO26 | SPST toggle to GND (Angle / Rate mode) |
| **Emergency Stop Button**| GPIO27 | Momentary pushbutton to GND |

## Serial Terminal Keyboard Controls
Open monitor at 115200 baud (`idf.py monitor` or `pio device monitor`):
* `[Spacebar]` : **EMERGENCY STOP** (Instantly disarms and zeroes motors)
* `[a]` : Arm / Disarm toggle switch
* `[w]` / `[s]` : Throttle Up / Down (5% steps)
* `[i]` / `[k]` : Pitch Forward / Back
* `[j]` / `[l]` : Roll Left / Right
* `[u]` / `[o]` : Yaw Left / Right
* `[x]` : Center all sticks (Roll=0, Pitch=0, Yaw=0)
* `[c]` : Send Sensor Calibration command (drone must be disarmed and motionless)
