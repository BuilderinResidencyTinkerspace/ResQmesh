# Week 6

**Goal this week:** Refine the KiCad circuit schematic and wiring layout reference, and follow up on pending component shipments.

## What we did

- Used the shipping downtime to review and refine the KiCad circuit design as an exact wiring blueprint:
  - **Circuit & Netlist Verification**: Double-checked pinouts for the Seeed Studio XIAO ESP32-S3, MPU9250 I2C lines, and AO3400A MOSFET gate/drain/source connections.
  - **Manual Wiring Layout Planning**: Mapped out the point-to-point wiring paths and perfboard layout in KiCad to keep motor power lines thick and isolated from sensitive signal traces.
  - **Protection & Passives Check**: Verified orientation of 1N5819 Schottky flyback diodes and placement of the 470µF bulk capacitor and gate pull-down resistors.
- Other than KiCad circuit refinements and planning the physical wiring layout, hands-on progress was limited while awaiting delivery of the ordered components.

## Problems and blockers

- **Extended Delivery Delays**: Component packages remained in transit with suppliers and couriers, preventing physical soldering, assembly, or live motor spin tests.
- Practical hardware assembly remained paused pending arrival of the physical components.

## Decisions

- **Circuit Design Only (No PCB Manufacturing)**: Confirmed that KiCad is used exclusively for circuit design, schematic capture, and wiring reference. No PCBs will be sent for commercial manufacturing; the circuit will be assembled via manual perfboard / point-to-point soldering directly on the drone frame to eliminate manufacturing lead times and keep the build strictly below the ₹3,000 budget.
- **Star-Ground Wiring Layout**: Planned physical wire routing so motor ground returns converge strictly at the battery negative solder pad, preventing voltage spikes on the MCU logic ground.

## Next week

- Take delivery of incoming electronics shipments and inspect components.
- Begin physical hand-soldering and wiring of the motor driver circuit following the KiCad blueprint.
- Validate power rails, test 3.3V logic switching with the XIAO ESP32-S3, and verify I2C communication with the IMU.

## Links

- Circuit Architecture & Driver Analysis: [code/ESP32-DRONE/README.md](file:///e:/ResQmesh/code/ESP32-DRONE/README.md#3-electrical--driver-circuit-analysis)
- Previous Log: [docs/week-05.md](week-05.md)
