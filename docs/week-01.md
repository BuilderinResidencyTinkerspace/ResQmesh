# Week 1

**Goal this week:** Brainstorm, define, and finalize the core idea, target applications, and architectural feasibility of a low-cost ESP32 swarm drone platform.

## What we did

- Brainstormed and pivoted the project concept to a versatile, low-cost micro quadcopter swarm platform capable of multi-domain applications.
- Researched and established 6 primary real-world application domains:
  1. **Emergency Response & Search and Rescue (SAR)**: Confined-space reconnaissance and ad-hoc communication relay.
  2. **Precision Agriculture & Forestry**: Crop canopy microclimate mapping and early forest fire detection.
  3. **Off-Grid & Tactical Mesh Networking**: Airborne ESP-NOW/LoRa packet repeaters.
  4. **Industrial & Indoor Infrastructure Inspection**: GPS-denied navigation in warehouses and ducting.
  5. **Swarm Robotics Research & Education**: Open, accessible testbed for multi-agent algorithms (Reynolds Boids, MARL).
  6. **Synchronized Aerial Light Displays**: Low-cost micro-drone formation flights with RGB LEDs.
- Defined a strict bill-of-materials (BOM) target of **< ₹3,000 per drone node** to ensure true affordability and scalability.
- Evaluated hardware feasibility around the Seeed Studio XIAO ESP32-S3, MPU9250 IMU, 720 coreless brushed motors, and 1S LiPo power.
- Restructured and documented the project vision, problem statement, and technical architecture in `docs/index.md`.

## Problems and blockers

- **Scope Definition**: Balancing multi-agent swarm capability with strict cost constraints (< ₹3,000 per node) without resorting to expensive flight controllers, brushless ESCs, or bulky GPS units.
- **Communication Topology**: Determining how drones communicate without heavy cellular or Wi-Fi router infrastructure (settled on ESP-NOW for low-latency P2P mesh and optional LoRa for long-range ground station links).

## Decisions

- **Swarm over Monolithic UAV**: Prioritized a distributed swarm of micro-drones over a single expensive drone to eliminate single-point-of-failure risks.
- **Microcontroller Selection**: Standardized on the ESP32-S3 (Seeed Studio XIAO form factor) for its compact footprint, dual-core 240 MHz compute, native Wi-Fi/ESP-NOW, and low cost.
- **Propulsion & Driver**: Selected 720 brushed coreless motors driven by AO3400A low-side MOSFETs to keep weight minimal and costs accessible.
- **Multi-Mission Modular Approach**: Designed the platform to be modular so sensors (IMU, ToF, barometers, environmental sensors) can be swapped depending on the specific application.

## Next week

- Finalize the component sourcing list and detailed bill of materials.

- Set up the firmware development environment in PlatformIO / ESP-IDF and outline the core flight stabilization tasks.

## Links

- Project Overview: [docs/index.md](index.md)
- Firmware & Hardware Codebase: [code/](file:///e:/ResQmesh/code)
