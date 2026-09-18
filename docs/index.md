# ResQmesh: Low-Cost ESP32 Swarm Drone Platform

An ultra-affordable, open-source micro quadcopter swarm platform powered by the ESP32 ecosystem, engineered for distributed sensing, mesh communication, and multi-domain aerial coordination.

**Team:** Jayasurya jayakumar, Name  
**Program:** Builder-in-Residence  

---

## The Problem

Commercial drone swarms and industrial UAV systems are prohibitively expensive—costing thousands of dollars per unit—and rely on proprietary, closed-source ecosystems. Moreover, conventional single-drone solutions suffer from critical single-point-of-failure risks: if one drone crashes or exhausts its battery, the entire mission is compromised.

Conversely, off-the-shelf hobby micro-drones lack multi-agent coordination capabilities, decentralized communication networks, and modular sensor integration. There is an urgent need for an **ultra-low-cost (<$25 per unit), open, and scalable micro-drone swarm platform** that democratizes multi-drone deployment across diverse industrial, environmental, emergency, and research applications.

---

## The Idea: Low-Cost ESP32 Swarm for Multiple Applications

**ResQmesh** transforms accessible micro-quadcopter hardware into an intelligent, collaborative aerial swarm powered by the **Seeed Studio XIAO ESP32-S3 / ESP32**. By combining deterministic onboard flight stabilization with high-speed peer-to-peer **ESP-NOW** communication and optional long-range **LoRa** telemetry, ResQmesh enables fleets of micro-drones to cooperate, share tasks, and self-heal in real time.

Instead of risking an expensive, monolithic aircraft, ResQmesh achieves operational resilience through redundancy and collective intelligence:
- **Dispensable & Scalable:** At a fraction of commercial costs, dozens of nodes can be fabricated and deployed simultaneously.
- **Decentralized Coordination:** Drones share state and telemetry directly via peer-to-peer ESP-NOW without requiring cellular towers, external Wi-Fi routers, or heavy ground infrastructure.
- **Adaptive Resilience:** If an individual drone lands, runs low on battery, or encounters an obstacle, the remaining swarm automatically redistributes spatial coverage and operational tasks.

---

## Key Applications

ResQmesh is designed as a versatile multi-mission platform capable of adapting across varied domains:

### 1. Emergency Response & Search and Rescue (SAR)
* **Hazardous Site Reconnaissance:** Penetrate collapsed structures, smoke-filled interiors, or narrow debris passages inaccessible to ground teams or large drones.
* **Survivor & Hazard Detection:** Swarm nodes sweep large geographic areas in parallel to locate victims, detect gas leaks, and map thermal hotspots.
* **Ad-Hoc Emergency Mesh:** Form an aerial communication chain over severed infrastructure, bridging first responders with ground command.

### 2. Precision Agriculture & Forestry
* **Microclimate & Canopy Mapping:** Multi-point real-time sampling of humidity, temperature, barometric pressure, and ambient light across crop fields.
* **Early Forest Fire & Hotspot Alerts:** Rapid aerial dispersal to detect early smoke signatures and localized heat spikes before fires spread.
* **Targeted Crop Scouting:** Distributed inspection of crop health, minimizing ground-patrol labor.

### 3. Off-Grid & Tactical Mesh Networking
* **Airborne Data Relays:** Act as dynamic, flying repeaters using ESP-NOW and LoRa to pass sensor packets over hills, dense foliage, or urban dead zones.
* **Sensor-to-Cloud Bridging:** Swarm nodes collect telemetry from scattered ground IoT nodes (e.g., environmental loggers) and relay data back to a central gateway.

### 4. Industrial & Indoor Infrastructure Inspection
* **GPS-Denied Navigation:** Inspect warehouse racking, ventilation shafts, tunnels, and factory ceilings where GPS signals are unavailable.
* **Multi-Angle Synchronized Perimeter Patrol:** Continuous, round-the-clock perimeter security through staged patrol shifts and automated battery rotation.

### 5. Swarm Robotics Research & Education
* **Accessible Swarm Testbed:** A disposable, low-risk testbed for universities and labs to validate multi-agent algorithms:
  * Decentralized flocking (Reynolds Boids) and formation flying.
  * Distributed consensus, collaborative SLAM, and dynamic obstacle avoidance.
  * Multi-Agent Reinforcement Learning (MARL) validation on real hardware.

### 6. Synchronized Aerial Light Shows & Displays
* **Miniature Swarm Choreography:** Coordinated, low-cost indoor aerial light formations and artistic displays using onboard high-brightness RGB LEDs.

---

## Technical Specifications & Architecture

| Parameter | Specification |
| :--- | :--- |
| **Flight Controller MCU** | Seeed Studio XIAO ESP32-S3 (Dual-core Xtensa LX7 @ 240 MHz, 2.4 GHz Wi-Fi / BLE 5) |
| **Inertial Measurement Unit** | InvenSense MPU9250 / MPU6050 (9-axis/6-axis accelerometer & gyroscope via 400 kHz I2C) |
| **Flight Loop Rate** | **500 Hz deterministic FreeRTOS loop** (2000 µs cycle) pinned to Core 1 |
| **Control Architecture** | Cascaded Dual-Loop PID (Outer Angle Loop + Inner Rate Loop with D-filter & anti-windup) |
| **Motor Drivers** | 4 × AO3400A N-Channel Logic-Level Trench MOSFETs ($R_{ds(\text{on})} < 30\text{ m}\Omega$) |
| **Propulsion System** | 4 × 720 Coreless Brushed DC Motors + 55mm Micro Propellers (Quad-X configuration) |
| **Power System** | 1S 3.7V LiPo battery (250–500 mAh) with onboard ADC voltage divider & low-battery alarm |
| **Inter-Drone Comms** | **ESP-NOW** (sub-5ms ultra-low latency peer-to-peer wireless packet transmission) |
| **Long-Range Telemetry** | Optional SX1262 / SX1276 LoRa module for kilometer-range ground station backhaul |
| **Safety Watchdog** | 200 ms signal loss failsafe timeout, auto-disarm, and roll/pitch limit gates |
| **Estimated Unit Cost** | < Rs 3000 per flight node |

---

## Hardware and Tools

### Boards & Actuators
* **Seeed Studio XIAO ESP32-S3** — Main flight controller and communication node.
* **MPU9250 / MPU6050** — High-rate IMU for attitude estimation and stabilization.
* **AO3400A MOSFET Drivers** — Low-side motor switches with 1N5819 flyback diodes and bulk decoupling.
* **720 Coreless Brushed DC Motors** — High thrust-to-weight micro propulsion.
* **1S LiPo Battery (3.7V)** — Lightweight power source with active ADC voltage monitoring.

### Sensors & Expansion Modules
* **Barometric Pressure Sensor (BMP280 / DPS310)** — Altitude hold and vertical estimation.
* **Time-of-Flight (VL53L1X / VL53L0X)** — Precision obstacle avoidance and ground distance ranging.
* **Environmental Sensors (SHT3x / BME280 / MQ-x)** — Temperature, humidity, and hazardous gas monitoring.
* **Optical Flow / Micro Camera (PMW3901 / ESP32-CAM)** — GPS-denied position hold and aerial inspection.
* **LoRa Transceiver (SX1262 / SX1278)** — Long-range low-power swarm-to-ground telemetry.

### Software Stack & Development Tools
* **PlatformIO & ESP-IDF** — Modular C++ flight controller and transmitter firmware.
* **FreeRTOS** — Dual-core deterministic real-time flight task and communication handling.
* **ESP-NOW** — Low-latency broadcast and unicast swarm messaging protocol.
* **Python Ground Station & Simulation** — Swarm telemetry processing, visual dashboards, and simulation injection.
* **Web UI Dashboard** — Real-time drone status, PID tuning, and swarm network health monitoring.
* **CAD (Fusion 360 / OpenSCAD)** — Custom 3D-printable lightweight micro-quad frames and motor mounts.

---

## Repo Layout

- `docs/` — Project documentation, technical specs, and weekly logs
- `code/` — Modular flight controller (`ESP32-DRONE`), transmitter (`ESP32-CONTROLLER`), and simulation tools
- `cad/` — 3D-printable airframe designs, motor guards, and hardware schematics

---

## Weekly Logs

Follow the development progress through our weekly milestone logs:

- [Week 1](week-01.md)
- [Week 2](week-02.md)
- [Week 3](week-03.md)
- [Week 4](week-04.md)
- [Week 5](week-05.md)
- [Week 6](week-06.md)
- [Week 7](week-07.md)
- [Week 8](week-08.md)
- [Week 9](week-09.md)
