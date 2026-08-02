# ResQmesh

A lightweight, low-cost drone swarm that creates temporary communication networks and collects environmental data in emergency or hard-to-reach areas.

**Team:** Name, Name
**Program:** Builder-in-Residence

## The problem

During natural disasters, accidents, and other emergency situations, communication infrastructure can become damaged or unavailable. First responders may have limited access to real-time information from hazardous, remote, or difficult-to-reach locations. Existing drone systems are often expensive, dependent on a single aircraft, and limited by flight time and coverage. When one drone fails, the mission may be interrupted or important information may be lost.

There is a need for a low-cost, scalable, and resilient system that can provide temporary connectivity and distributed environmental awareness when conventional infrastructure is unavailable.

## The idea

ResQmesh is a lightweight, ESP32-based drone swarm designed to operate as a distributed aerial communication and sensing network. Each drone functions as a mobile node that can share status, relay information, and collect environmental data while coordinating with other drones and a ground station.

Unlike a conventional single-drone system, ResQmesh distributes tasks across multiple low-cost aerial units. The swarm can expand coverage, reduce dependence on a single drone, and adapt when an individual node becomes unavailable. ESP-NOW can support fast local communication between nearby drones, while LoRa can provide long-range, low-power telemetry and communication.

The initial prototype will demonstrate coordinated communication, environmental sensing, and basic adaptive task allocation in a controlled environment. The long-term goal is to explore how lightweight drone swarms can support disaster response, emergency assessment, and temporary communication in areas where existing infrastructure is damaged or inaccessible.

## Hardware and tools

## Boards / Sensors

* **ESP32 development board** — swarm communication, sensor integration, and coordination logic
* **LoRa module** — long-range, low-power telemetry and communication
* **IMU (accelerometer + gyroscope)** — motion and orientation sensing
* **Barometric pressure sensor** — altitude estimation
* **GPS module** *(optional for outdoor testing)* — position and navigation
* **Time-of-Flight or ultrasonic sensor** *(optional)* — obstacle and distance detection
* **Temperature and humidity sensor** — environmental monitoring
* **Air-quality or gas sensor** *(optional)* — detecting environmental conditions
* **Battery voltage/current sensor** — battery health and power monitoring

## Other Tools

* **PlatformIO or Arduino IDE** — ESP32 firmware development
* **ESP-NOW** — low-latency communication between nearby drones
* **LoRa communication library** — long-range telemetry and data exchange
* **MQTT** *(optional)* — communication between the ground station and monitoring dashboard
* **Python** — swarm simulation, data processing, and ground-station tools
* **Web dashboard** — live monitoring of drone status, sensor data, and network health
* **GitHub** — version control, documentation, and collaboration
* **CAD software** *(optional)* — lightweight frame and component design
* **Drone simulation software** *(optional)* — testing swarm behavior before flight


## Repo layout

- `docs/` — this site: overview and weekly logs
- `code/` — firmware, scripts, anything runnable
- `cad/` — 3D models and design files

## Weekly logs

Start with [Week 1](week-01.md) and fill one in each week.
