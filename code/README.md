# Drone Firmware & Transmitter Subsystems

This directory contains the complete modular firmware for the quadcopter and its ground controller:

## Subsystems

1. **[ESP32-DRONE](ESP32-DRONE/)**:
   - Production flight controller firmware for **Seeed Studio XIAO ESP32-S3** and **MPU9250 IMU**.
   - Deterministic **500 Hz FreeRTOS flight loop** pinned to Core 1.
   - Cascaded Angle + Rate PID with derivative filtering and anti-windup.
   - Quad-X dynamic anti-saturation mixer driving 4 × 720 coreless motors via AO3400A MOSFETs.
   - ESP-NOW wireless receiver with CRC16 and 200 ms failsafe timeout watchdog.
   - 1S LiPo ADC battery monitor and alarm state machine.
   - Interactive USB-UART debugging CLI and simulation injection mode.
   - Automated native unit test suite with 41 passing algorithmic assertions.

2. **[ESP32-CONTROLLER](ESP32-CONTROLLER/)**:
   - Ground transmitter controller firmware for ESP32 / ESP32-S3.
   - Transmits packed `ControlPacket` over ESP-NOW at 50 Hz.
   - Supports both physical analog RC gimbals and interactive serial terminal keyboard controls (`W/S/A/D/Space`).
