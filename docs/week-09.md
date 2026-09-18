# Week 9

**Goal this week:** Fabricate, assemble, and bench-test a custom, highly efficient 4-channel MOSFET motor driver module for the 720 coreless motors and XIAO ESP32-S3.

## What we did

- Built and hand-soldered a custom 4-channel micro motor driver board tailored specifically for 1S micro quadcopter propulsion:
  - **Ultra-Efficient MOSFET Switching**: Utilized **AO3400A** N-channel trench MOSFETs featuring exceptionally low on-resistance ($R_{ds(\text{on})} < 30\text{ m}\Omega$ at 3.3V logic level). At full 1.5A motor current, conduction loss is merely $P = I^2 R = (1.5)^2 \times 0.030 \approx 0.067\text{ W}$, achieving $>98\%$ electrical power efficiency and running completely cool to the touch without heatsinks.
  - **Gate Drive & Boot Safety**: Added 100Ω series gate resistors to dampen $LC$ gate ringing and protect ESP32-S3 GPIOs from inrush spikes, combined with 10kΩ gate pull-downs ensuring motors remain strictly locked off during MCU boot and firmware flashing.
  - **Inductive Back-EMF Clamping**: Installed ultra-fast 1N5819 Schottky flyback diodes antiparallel across each motor output to clamp inductive spikes ($V = -L \frac{di}{dt}$) safely to $V_{bat} + 0.45\text{ V}$.
  - **Power Bus Stabilization**: Integrated a 470µF low-ESR bulk electrolytic capacitor across the main 1S LiPo power rail, preventing battery voltage sag and microcontroller brownouts during full 8A four-motor burst punches.
  - **RF Noise Decoupling**: Soldered 100nF ceramic decoupling capacitors directly across the motor solder tabs to shunt brush commutation RF arcing before it reaches the MPU9250 I2C bus.
- Driven via **20 kHz Ultrasonic Hardware PWM**: Configured ESP32-S3 LEDC PWM timers at 20 kHz with 10-bit resolution (0–1023 duty), eliminating audible motor whine while maximizing smooth torque delivery.
- Conducted comprehensive bench testing via the interactive USB serial CLI:
  - Verified individual motor test commands (`test_motor 1 5` through `test_motor 4 5`).
  - Measured drain-to-source voltage drop ($V_{ds} \approx 45\text{ mV}$ at 1.5A), confirming complete MOSFET saturation and high electrical efficiency.
  - Verified proper Quad-X motor rotation directions: M1 (FL) CW, M2 (FR) CCW, M3 (RR) CW, M4 (RL) CCW.

## Problems and blockers

- **Compact SOT-23 Soldering**: Soldering miniature SOT-23 AO3400A packages by hand required fine-gauge solder, flux, and high magnification to prevent accidental bridging between Gate, Drain, and Source pins.
- **Motor Brush Commutation Noise**: Initial motor spin caused high-frequency electrical hash on nearby wires; resolved by twisting motor lead pairs and placing 100nF ceramic caps directly across the motor terminals.

## Decisions

- **Direct Low-Side Switching**: Chose AO3400A low-side switching topology directly driven by 3.3V GPIOs over complex brushless ESCs, drastically reducing both BOM cost (< ₹3,000 budget) and component weight.
- **Star Grounding Architecture**: Implemented a dedicated star-ground topology where motor power ground returns converge strictly at the battery negative terminal, isolating high-current transients from the sensitive IMU sensor logic.
- **20 kHz PWM Frequency**: Locked the motor switching frequency at 20 kHz to eliminate human-audible frequency buzz while keeping MOSFET dynamic switching losses negligible.

## Next week

- Complete final mechanical integration: mount the custom motor driver, XIAO ESP32-S3, and IMU firmly onto the 3D-printed unibody micro-X frame.
- Establish live wireless pilot control via ESP-NOW from the ground transmitter.
- Conduct low-throttle tethered hover tests and tune the cascaded PID attitude control loops.

## Links

- Circuit Architecture & Driver Analysis: [code/ESP32-DRONE/README.md](file:///e:/ResQmesh/code/ESP32-DRONE/README.md#3-electrical--driver-circuit-analysis)
- Motor Driver Implementation: [code/ESP32-DRONE/firmware/motors.cpp](file:///e:/ResQmesh/code/ESP32-DRONE/firmware/motors.cpp)
- Project Overview: [docs/index.md](index.md)
