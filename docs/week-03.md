# Week 3

**Goal this week:** Finalize the complete hardware bill of materials (BOM), verify component compatibility against the < ₹3,000/node budget, and place procurement orders for all drone and controller parts.

## What we did

- Researched, benchmarked, and finalized the complete hardware component list for the flight nodes and ground controller:
  - **Microcontroller**: Seeed Studio XIAO ESP32-S3 (dual-core 240 MHz, compact footprint, native ESP-NOW).
  - **Sensors**: MPU9250 / MPU6050 6/9-axis IMU modules for high-rate I2C attitude tracking.
  - **Motors & Props**: 720 (7×20mm) high-RPM coreless DC motors (CW and CCW matched pairs) with 55mm high-efficiency propellers.
  - **Motor Driver Components**: AO3400A logic-level N-channel MOSFETs, 1N5819 Schottky flyback protection diodes, 100Ω gate resistors, and 10kΩ pull-downs.
  - **Power & Filtering**: 1S 3.7V 300–450 mAh LiPo batteries, 470µF bulk low-ESR electrolytic capacitors, and 100nF motor noise decoupling ceramic capacitors.
  - **Transmitter Components**: ESP32 module with analog joystick gimbals and serial interface.
- Verified unit economics: totaled component costs to ensure each flight node stays strictly under the **₹3,000** target budget.
- Placed purchase orders with suppliers for all structural, electrical, and electronic components (including spare motors and props for crash resilience).

## Problems and blockers

- **Vendor Availability & Lead Times**: Variations in shipping times across multiple electronics suppliers; had to cross-compare vendors to minimize delays.
- **Weight vs. Capacity Trade-Off**: Ensuring ordered 1S LiPo batteries did not exceed 10–12g so the 720 motors retain sufficient thrust-to-weight margin (> 1.8:1) for stable hover.
- **Sensor Authenticity**: Sourcing reliable MPU9250/MPU6050 breakout boards with clean I2C communication characteristics.

## Decisions

- **Direct 3.3V Logic MOSFET Drive**: Confirmed the AO3400A FET ($V_{gs(\text{th})} < 1.45\text{V}$, $R_{ds(\text{on})} < 30\text{ m}\Omega$) can be driven directly by the ESP32-S3 GPIOs, saving cost and board space by omitting dedicated gate driver ICs.
- **Ordered Spares**: Ordered extra 720 motors, propellers, and passive components to support rapid iteration and inevitable bench testing wear-and-tear.
- **1S LiPo Standard**: Standardized on 1S 3.7V batteries with Molex/JST connectors for safe, uniform charging across the swarm fleet.

## Next week

- Set up the firmware development environment in PlatformIO / ESP-IDF.
- Develop and simulate modular firmware architecture (sensor drivers, FreeRTOS dual-core task design, attitude filter, and PID algorithms) while awaiting shipment delivery.

## Links

- Project Overview & Specifications: [docs/index.md](index.md)
- Firmware Codebase: [code/](file:///e:/ResQmesh/code)
- Airframe CAD: [cad/](file:///e:/ResQmesh/cad)
