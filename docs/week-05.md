# Week 5

**Goal this week:** Learn KiCad EDA software, design the custom schematic, and layout the flight controller / motor driver circuit for the micro-quadcopter.

## What we did

- Studied KiCad EDA workflows, including schematic capture (Eeschema), custom symbol/footprint creation, netlist generation, and PCB design rules.
- Created custom schematic symbols and verified footprints for:
  - **Seeed Studio XIAO ESP32-S3** pin headers.
  - **MPU9250 / MPU6050** 400 kHz I2C breakout.
  - **AO3400A** N-channel MOSFETs in SOT-23 packaging.
- Designed the full schematic for the quadcopter power and driver board:
  - **4x Motor Driver Channels**: Low-side AO3400A switches with 100Ω series gate resistors to dampen ringing and 10kΩ gate pull-downs to prevent floating states at boot.
  - **Inductive Kick Protection**: 1N5819 Schottky diodes antiparallel across motor terminals to safely clamp back-EMF flyback voltage spikes.
  - **Battery Voltage Monitoring**: A 2:1 resistive divider (100kΩ / 100kΩ) stepping down the 4.2V max LiPo voltage to 2.1V for XIAO D0 (GPIO1 / ADC1_CH0).
  - **Power Decoupling & Filtering**: Filter pads for a 470µF bulk capacitor across the battery rail and 100nF high-frequency ceramic caps across motor leads.
- Applied star-grounding architecture in the schematic to isolate motor return paths from sensitive IMU digital/analog grounds.
- Ran Electrical Rules Check (ERC) in KiCad to verify net connections and validate pin electrical types.

## Problems and blockers

- **KiCad Learning Curve**: Getting familiar with footprint assignment, pin numbering conventions (verifying SOT-23 pinout: Gate=1, Source=2, Drain=3), and net labeling.
- **Noise & Ground Bounce Mitigation**: Ensuring the inductive flyback from 4 brushed motors pulsing at 20 kHz would not cause voltage sag or freeze the ESP32-S3 core.

## Decisions

- **Star Grounding Scheme**: Tied motor source ground returns and MCU logic grounds together strictly at a single point (battery negative terminal) to prevent ground bounce on the I2C bus.
- **Hardware Failsafe Gate Resistors**: Added 10kΩ pull-downs on all 4 gate lines so motors remain completely off during firmware flashing or microcontroller reboot.
- **Circuit Design for Hand Assembly (No PCB Manufacturing)**: Decided to use KiCad strictly for schematic design, circuit validation, and wiring layout reference. The physical circuit will be hand-soldered on perfboard/point-to-point to keep per-node costs strictly under ₹3,000 and avoid manufacturing turnaround delays.

## Next week

- Assemble and solder the physical motor driver circuit according to the KiCad schematic.
- Inspect solder joints, check for shorts with a multimeter, and verify stable 3.3V logic levels.
- Perform initial bench test of single-channel and four-channel motor PWM switching with the XIAO ESP32-S3.

## Links

- Circuit Architecture & Schematic Analysis: [code/ESP32-DRONE/README.md](file:///e:/ResQmesh/code/ESP32-DRONE/README.md#3-electrical--driver-circuit-analysis)
- Project Overview: [docs/index.md](index.md)
