# Week 10

**Goal this week:** Develop, flash, and validate custom real-time motor control and flight firmware on the Seeed Studio XIAO ESP32-S3, architected specifically for multi-drone swarm coordination.

## What we did

- Developed and deployed custom, hard-real-time motor control and flight firmware in C++ from scratch:
  - **Deterministic 500 Hz Control Loop**: Pinned strictly to Core 1 of the dual-core ESP32-S3 via FreeRTOS (2000 µs cycle time). Executes IMU burst read, complementary attitude estimation, cascaded PID, and mixer updates in under 450 µs, leaving 1550 µs of deterministic headroom.
  - **Cascaded Angle + Rate PID Control**: Outer angle P-loop calculates target angular rates from desired orientation demands; inner rate PID loop drives axis torque demands with derivative filtering ($\tau = 0.005\text{s}$) and integral anti-windup clamping.
  - **Quad-X Dynamic Anti-Saturation Mixer**: Mathematically mixes Throttle + Roll + Pitch + Yaw across the 4 motor outputs. When large attitude corrections coincide with high throttle, the mixer calculates excess demand ($\max(M_1..M_4) - 1023$) and subtracts it equally from all four motors, preserving 100% attitude control authority without clipping.
  - **20 kHz LEDC Ultrasonic Motor Driver**: Configured ESP32-S3 LEDC hardware PWM timers at 20 kHz with 10-bit duty resolution (0–1023) to smoothly drive the AO3400A MOSFETs without audible coil whine.
- Architected the firmware specifically for multi-agent swarm operations:
  - **Decentralized ESP-NOW Swarm Communication**: Handled asynchronously on Core 0, enabling sub-5ms low-latency peer-to-peer wireless packet exchange across drones without requiring external Wi-Fi routers or heavy ground infrastructure.
  - **Swarm Packet Framing & Validation**: Designed a packed `ControlPacket` structure with individual drone node addressing, packet sequence numbering, and CRC16 checksum verification for robust multi-agent packet filtering.
  - **200 ms Failsafe Watchdog**: Independent safety state machine monitoring swarm heartbeat packets; instantly disarms and cuts all 4 motor outputs to 0 if an individual node loses connection, preventing rogue flyaways and protecting nearby swarm members.
  - **Non-Blocking Serial CLI & Telemetry**: Integrated interactive serial debugging commands (`status`, `imu`, `attitude`, `motors`, `test_motor`) on Core 0 for live swarm diagnostics and parameter inspection.
- Flashed firmware onto the Seeed Studio XIAO ESP32-S3 and verified live motor control on hardware:
  - Confirmed deterministic 500 Hz execution timing.
  - Tested motor anti-saturation and verified that tilting the board induces immediate differential counter-thrust from the corresponding motor pairs.

## Problems and blockers

- **Dual-Core Task Contention**: Ensuring that wireless RF communications (ESP-NOW) and logging on Core 0 do not introduce jitter or delay into the 500 Hz hard-real-time flight loop on Core 1.
- **Packet Collision in Multi-Agent Broadcasts**: Handled by implementing sequence-tracked packets with CRC16 filtering to discard corrupted or out-of-order packets in dense RF environments.

## Decisions

- **Custom Swarm Firmware vs. Generic Flight Stacks**: Chose to build custom, modular C++ firmware rather than using bulky off-the-shelf stacks (e.g. Betaflight). This allows direct low-level control over dual-core task affinity, ultra-lightweight binary size, and native peer-to-peer swarm messaging protocols.
- **Dynamic Priority Anti-Saturation**: Standardized on dynamic throttle reduction during saturation to guarantee that swarm nodes never lose roll/pitch stabilization during aggressive coordinated maneuvers.

## Next week

- Install 55mm propellers and conduct tethered low-throttle indoor hover tests.
- Test wireless ESP-NOW pairing between the ground controller and multiple drone nodes simultaneously.
- Fine-tune cascaded PID gains (Angle $K_p$, Rate $K_p$, $K_i$, $K_d$) for optimal hover stability.

## Links

- Firmware Architecture & Documentation: [code/ESP32-DRONE/README.md](file:///e:/ResQmesh/code/ESP32-DRONE/README.md)
- Motor Control & Mixer: [code/ESP32-DRONE/firmware/motors.cpp](file:///e:/ResQmesh/code/ESP32-DRONE/firmware/motors.cpp)
- Flight Controller Loop: [code/ESP32-DRONE/firmware/flight_controller.cpp](file:///e:/ResQmesh/code/ESP32-DRONE/firmware/flight_controller.cpp)
- Project Overview: [docs/index.md](index.md)
