# Week 7

**Goal this week:** Develop, execute, and validate native host flight simulations and automated algorithmic unit tests to verify the 500 Hz flight control pipeline before flashing hardware.

## What we did

- Developed a standalone C++ host simulation framework (`simulate_flight.cpp`) and automated unit test suite (`run_all_tests.cpp`) to validate flight dynamics natively on PC:
  - **Simulation Runner (`simulate_flight.cpp`)**: Runs the full 500 Hz control loop ($dt = 2000\ \mu\text{s}$) with simulated IMU dynamics and virtual ESP-NOW telemetry packets.
  - **Scenario 1 (Boot & Disarm)**: Confirmed that motors remain strictly at 0 PWM and system boots safely into `DISARMED` state.
  - **Scenario 2 (Pre-Arm Safety Interlock)**: Verified arming only engages when pilot throttle is at 0 and tilt is within allowable limits ($< 25^\circ$).
  - **Scenario 3 (Active Hover)**: Tested steady-state level hover at 40% throttle (400 PWM), confirming balanced motor thrust across M1–M4.
  - **Scenario 4 (Wind Disturbance Correction)**: Injected an external +15° nose-up pitch disturbance; verified the PID controller promptly increased front motors (M1/M2) and decreased rear motors (M3/M4) to restore level flight.
  - **Scenario 5 (Pilot Roll Response)**: Injected pilot roll commands (+20° bank) and verified differential torque response from the Quad-X mixer.
  - **Scenario 6 (Anti-Saturation at 95% Throttle Punch)**: Simulated full throttle punch with roll demand; verified the anti-saturation algorithm dynamically lowered collective throttle to prevent PWM clipping (>1023), preserving full attitude control authority.
  - **Scenario 7 (Signal Loss Failsafe)**: Injected a 250 ms radio dropout (>200 ms watchdog limit); verified immediate transition to `FAILSAFE` and instantaneous motor cutoff.
- Executed the automated unit test suite with 41 algorithmic assertions covering the cascaded PID, attitude complementary filter, CRC16 packet checksums, and ADC moving-average filter.
- Created `run_simulation.bat` and `run_tests.bat` scripts for automated one-click host builds.

## Problems and blockers

- **Host-Embedded Abstraction**: Emulating ESP-IDF hardware peripherals (LEDC PWM timers, hardware ADC, and ESP-NOW radio) on host Windows/Linux environments without hardware attached.
- **Derivative Kick Mitigation**: High-frequency noise on pilot stick step inputs causing derivative spikes in the PID loop; verified and tuned the low-pass D-filter time constant ($\tau = 0.005\text{s}$) in simulation.

## Decisions

- **Hardware Abstraction for Simulation**: Implemented simulation injection hooks in `imu.h` and `receiver.h` to allow virtual sensor telemetry to be fed into the exact production control loop.
- **Host Testing First**: Adopted a strict "test in simulation before testing on hardware" policy to minimize physical crash risks and verify failsafes before spinning real propellers.

## Next week

- Unbox incoming hardware components and assemble the electronics on the micro-quadcopter frame.
- Flash the firmware onto the Seeed Studio XIAO ESP32-S3.
- Perform physical bench testing with live IMU sensor readings and verify motor rotation directions.

## Links

- Simulation Runner: [code/ESP32-DRONE/tests/simulate_flight.cpp](file:///e:/ResQmesh/code/ESP32-DRONE/tests/simulate_flight.cpp)
- Algorithmic Unit Tests: [code/ESP32-DRONE/tests/run_all_tests.cpp](file:///e:/ResQmesh/code/ESP32-DRONE/tests/run_all_tests.cpp)
- Firmware & Testing Guide: [code/ESP32-DRONE/README.md](file:///e:/ResQmesh/code/ESP32-DRONE/README.md#8-build-flash-and-testing-instructions)
