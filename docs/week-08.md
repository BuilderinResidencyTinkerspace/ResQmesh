# Week 8

**Goal this week:** Receive and inspect arrived hardware components, re-evaluate communication architecture, begin physical assembly, and test the IMU sensor with the XIAO ESP32-S3.

## What we did

- Received and unboxed the electronics shipments:
  - Seeed Studio XIAO ESP32-S3 microcontrollers.
  - InvenSense MPU9250 / MPU6050 9-axis/6-axis IMU breakout boards.
  - 720 (7×20mm) coreless brushed DC motors and 55mm propellers.
  - AO3400A N-channel MOSFETs, 1N5819 Schottky diodes, and passive resistors/capacitors.
  - 1S 3.7V LiPo batteries (verified nominal storage voltage at ~3.82V).
- Re-evaluated long-range LoRa integration:
  - Initially planned to incorporate onboard LoRa modules for long-range telemetry.
  - Found that an ultra-compact ESP32-S3 board with integrated LoRa was unavailable from suppliers in the required micro form factor.
  - Adding discrete external LoRa breakout modules would add unacceptable payload weight (> 6–8g) for a 1S micro-drone, severely degrading flight time.
  - Decided to defer LoRa as a future upgrade and focus primary swarm peer-to-peer communications and telemetry on high-speed **ESP-NOW**.
- Initiated physical assembly and validated the IMU subsystem:
  - Wired the MPU9250 IMU to the Seeed Studio XIAO ESP32-S3 over 400 kHz Fast-Mode I2C (`SDA: GPIO5 / D4`, `SCL: GPIO6 / D5`).
  - Successfully verified device communication and `WHO_AM_I` register response at address `0x68`.
  - Streamed live 3-axis accelerometer and 3-axis gyroscope data through the interactive USB serial console.
  - Executed a 500-sample stationary zero-bias calibration, computing gyro drift offsets ($< 0.1^\circ/\text{s}$).
  - Tested the complementary attitude filter with physical board rotations, confirming responsive, stable roll and pitch angle tracking.

## Problems and blockers

- **LoRa Module Sourcing**: Inability to source an ultra-compact, lightweight ESP32-S3 board with onboard LoRa without exceeding the micro-quadcopter's weight budget.
- **I2C Signal Integrity**: Initial intermittent I2C bus hangs due to weak internal pull-up resistors; resolved by placing external 4.7kΩ pull-up resistors on the SDA and SCL lines to 3.3V.

## Decisions

- **LoRa as a Future Feature**: Shifted LoRa to a planned future expansion phase (such as for larger outdoor drone builds or dedicated ground-relay stations). The current swarm drone platform will leverage native **ESP-NOW** for sub-5ms low-latency peer-to-peer mesh coordination.
- **IMU Orientation & Placement**: Fixed the IMU board layout in line with the forward flight axis on the airframe to avoid software axis re-mapping complications.

## Next week

- Hand-solder the 4-channel AO3400A MOSFET motor driver stage with flyback protection diodes.
- Mount the XIAO ESP32-S3, IMU, and 720 coreless motors onto the 3D-printed unibody micro-X frame.
- Perform live motor rotation direction checks and bench-test throttle response without propellers.

## Links

- IMU Driver Implementation: [code/ESP32-DRONE/firmware/imu.cpp](file:///e:/ResQmesh/code/ESP32-DRONE/firmware/imu.cpp)
- Pinout & Hardware Architecture: [code/ESP32-DRONE/README.md](file:///e:/ResQmesh/code/ESP32-DRONE/README.md#2-hardware-pinout-seeed-studio-xiao-esp32-s3)
- Project Overview: [docs/index.md](index.md)
