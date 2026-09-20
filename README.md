# ResQmesh: Autonomous Ultra-Low-Cost ESP32 Swarm Drone Platform

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Hardware Target](https://img.shields.io/badge/Hardware-Seeed%20Studio%20XIAO%20ESP32--S3-00C49F.svg)](https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html)
[![Flight Loop](https://img.shields.io/badge/Flight%20Loop-500%20Hz%20Deterministic-blue.svg)](#the-500-hz-deterministic-flight-stack-core-1)
[![Swarm Mesh](https://img.shields.io/badge/Swarm%20Comms-ESP--NOW%20%28%3C5ms%29-purple.svg)](#the-swarm-mesh-network-esp-now--phone-softap)
[![Unit Tests](https://img.shields.io/badge/Host%20Tests-73%2F73%20Passing-brightgreen.svg)](#automated-testing--flight-simulation)
[![Unit Cost](https://img.shields.io/badge/Unit%20Cost-%3C%20%E2%82%B92%2C250%20%2F%20%2425-orange.svg)](#complete-bill-of-materials-bom--budget)

*An ultra-affordable (<$25 / ₹2,224), open-source micro-quadcopter swarm platform powered by the ESP32-S3 ecosystem, engineered for distributed sensing, ad-hoc emergency mesh communication, and collaborative autonomous aerial coordination.*

</div>

---

## Table of Contents

1. [The Story: Why ResQmesh Exists](#the-story-why-resqmesh-exists)
   - [The Monolithic Problem](#the-monolithic-problem)
   - [The ₹3,000 Swarm Gamble](#the-3000-swarm-gamble)
   - [The Workbench Battles: Snapped Arms, Smoky FETs, and Motor EMI](#the-workbench-battles)
   - [The Breakthrough: "It Fights Back!"](#the-breakthrough-it-fights-back)
2. [System Specifications & Architecture](#system-specifications--architecture)
3. [Complete Bill of Materials (BOM) & Budget](#complete-bill-of-materials-bom--budget)
4. [Hardware Schematic & Wiring Blueprint](#hardware-schematic--wiring-blueprint)
   - [Driver Circuit Design & MOSFET Selection](#driver-circuit-design--mosfet-selection)
   - [EMI Mitigation: Twisted Pairs & Star Grounding](#emi-mitigation-twisted-pairs--star-grounding)
   - [XIAO ESP32-S3 Pin Mapping](#xiao-esp32-s3-pin-mapping)
5. [Airframe 3D Printing & Fabrication Guide](#airframe-3d-printing--fabrication-guide)
   - [The 0.12mm Slicer Shrinkage Fix](#the-012mm-slicer-shrinkage-fix)
   - [Weight Budget Breakdown](#weight-budget-breakdown)
6. [Firmware Architecture](#firmware-architecture)
   - [Dual-Core FreeRTOS Task Allocation](#dual-core-freertos-task-allocation)
   - [The 500 Hz Deterministic Flight Stack (Core 1)](#the-500-hz-deterministic-flight-stack-core-1)
   - [Dynamic Priority Anti-Saturation Mixer](#dynamic-priority-anti-saturation-mixer)
   - [Cascaded Dual-Loop PID Controller](#cascaded-dual-loop-pid-controller)
7. [The Swarm Mesh Network: ESP-NOW + Phone SoftAP](#the-swarm-mesh-network-esp-now--phone-softap)
   - [Concurrent Dual-Mode Radio Operation](#concurrent-dual-mode-radio-operation)
   - [Piloting with Any Smartphone (Zero-Install Touch UI)](#piloting-with-any-smartphone-zero-install-touch-ui)
   - [Decentralized Swarm Packet Framing](#decentralized-swarm-packet-framing)
8. [Step-by-Step Replication Guide (Zero to First Flight)](#step-by-step-replication-guide)
   - [Step 1: 3D Printing the Airframe](#step-1-3d-printing-the-airframe)
   - [Step 2: Soldering the MOSFET Driver Board](#step-2-soldering-the-mosfet-driver-board)
   - [Step 3: Motors, Decoupling & Harness Assembly](#step-3-motors-decoupling--harness-assembly)
   - [Step 4: Toolchain Setup & Firmware Flashing](#step-4-toolchain-setup--firmware-flashing)
   - [Step 5: Automated Testing & Verification](#step-5-automated-testing--verification)
   - [Step 6: Safe Commissioning Checklist (Props OFF!)](#step-6-safe-commissioning-checklist-props-off)
   - [Step 7: Tethered Hover & Initial Flight](#step-7-tethered-hover--initial-flight)
9. [Desktop Ground Control Station & Tools](#desktop-ground-control-station--tools)
10. [Repository Layout](#repository-layout)
11. [License & Acknowledgments](#license--acknowledgments)

---

## The Story: Why ResQmesh Exists

### The Monolithic Problem
Modern commercial drone swarms and industrial UAVs are marvelous feats of engineering—and disastrously expensive. At $2,000 to $10,000 per unit, they rely on proprietary radio protocols, heavy STM32 architectures, complex RTK GPS beacons, and centralized ground command stations. 

Worst of all, conventional aerial operations suffer from a **single-point-of-failure**: if one multi-thousand-dollar drone clips a branch, runs low on battery, or suffers radio jamming in a collapsed structure, the entire mission grinds to a halt.

Meanwhile, off-the-shelf toy micro-drones are cheap ($20–$30), but their hardware is locked down. They lack inter-agent communication, offer no access to raw flight dynamics, and cannot coordinate or share sensor telemetry.

### The ₹3,000 Swarm Gamble
We asked a deceptively simple question:  
*Can we build an open-source, resilient, autonomous micro-drone swarm platform where each unit costs **less than ₹2,500 ($25)**, weighs **under 35 grams**, and runs a deterministic **500 Hz hard-real-time flight loop** alongside **peer-to-peer mesh radio**?*

Ten weeks ago, this was just numbers scrawled across a workbench whiteboard. To hit this threshold, off-the-shelf flight stacks like Betaflight or ArduPilot were non-starters: they are built around dedicated RC receivers (ELRS/CRSF), expect STM32 hardware, and do not accommodate low-overhead, peer-to-peer swarm messaging on ESP32 silicon. 

We had to write our own flight controller and mesh communication stack from scratch in C++.

### The Workbench Battles

#### 1. Snapped Arms & Slicer Shrinkage
Before you write a single line of flight code, your drone must obey Newton. Our early 5.0g skeletal 3D-printed frame flexed like a wet noodle when the motors spooled up. Under full thrust, the motor pods deflected by nearly 3°, causing the IMU to measure the bending of the plastic instead of the attitude of the aircraft. 

When we shifted to a reinforced unibody design, our 7.00mm motor bores came off the 3D printer measuring 6.85mm due to thermal plastic shrinkage. Pressing a motor in by hand resulted in a sickening *snap* down the layer lines. The solution wasn't buying expensive carbon fiber; it was dialing in **`+0.12mm Horizontal Hole Expansion`** in the slicer, yielding a rigid 7.4g unibody with an exact 7.02mm friction fit.

#### 2. The SOT-23 MOSFET Discovery
Most DIY drone projects try to drive standard power MOSFETs directly from 3.3V microcontroller pins, only to discover that the FETs never fully turn on. Standard gates require 4.5V to 10V to reach saturation. Driven with 3.3V, they act like high-resistance heaters, drop the battery voltage, and toast themselves.

Digging deep into datasheets, we uncovered the **AO3400A N-Channel MOSFET** in an ultra-compact SOT-23 package. With a gate threshold voltage ($V_{gs(\text{th})}$) between **0.65V and 1.45V**, it is slammed completely into full saturation by the ESP32-S3's 3.3V logic. At $R_{ds(\text{on})} < 30\text{ m}\Omega$, it switches 2A of motor current with virtually zero thermal dissipation, eliminating the weight and cost of dedicated gate driver ICs.

#### 3. Taming Motor EMI Noise
Brushed coreless motors at 50,000 RPM are miniature spark transmitters. Mechanical brush commutation produces violent micro-arcing, radiating electromagnetic noise straight into low-voltage sensor lines. Our first physical wiring run threw random I2C bus lockups on the MPU9250.

We cured this with three physical design rules:
- **Tightly Twisted Motor Pairs:** Twisting each motor's power leads (4 turns/cm) creates opposing magnetic fields that cancel radiated RF hash.
- **100nF Ceramic Tab Capacitors:** Soldered directly across each motor's solder tabs to shunt RF arcing right at the spark source.
- **Star Grounding:** Routing high-current motor return paths directly to the battery negative pad, physically isolating them from the microcontroller's logic ground.

### The Breakthrough: "It Fights Back!"
In Week 10, all subsystems converged. We held the 32.2g quadcopter loosely between two fingers, armed the flight controller, and bumped the throttle to 25%. The four 55mm props spun into a quiet, vibration-free blur at 20 kHz ultrasonic PWM.

When we pushed the nose down with a finger, the front two motors instantly surged with increased RPM while the rear two backed off, delivering a firm, gyroscopic counter-torque right into our fingertips. When we tilted it left, the left motors roared to restore level flight.

It felt alive. The hardware worked, the electrical efficiency exceeded 98%, the dual-core FreeRTOS architecture maintained a rock-solid 500 Hz flight loop, and the total build cost totaled **₹2,224 ($24.80)**.

---

## System Specifications & Architecture

```
+-------------------------------------------------------------------------------+
|                      ResQmesh System Specifications                           |
+------------------------------+------------------------------------------------+
| Parameter                    | Value / Description                            |
+------------------------------+------------------------------------------------+
| Flight Controller MCU        | Seeed Studio XIAO ESP32-S3 (Dual-Core @ 240MHz)|
| Inertial Measurement Unit    | InvenSense MPU9250 / MPU6500 (400 kHz Fast I2C)|
| Flight Loop Execution Rate   | 500 Hz deterministic (2000 µs FreeRTOS cycle)  |
| Attitude Estimation          | Complementary Filter (α = 0.980, 500 Hz fusion)|
| Motor Drive PWM              | 20 kHz Ultrasonic LEDC (10-bit resolution)     |
| Low-Side Power Switches      | 4 × AO3400A Logic-Level N-Channel MOSFETs      |
| Propulsion System            | 4 × 720 Coreless Brushed DC + 55mm Propellers  |
| Power System                 | 1S 3.7V 380 mAh LiPo Battery (25C discharge)   |
| Inter-Drone Mesh Radio       | ESP-NOW 2.4 GHz Peer-to-Peer (sub-5ms latency) |
| Direct Mobile Link           | Autonomous Wi-Fi SoftAP + WebSockets (40 Hz)   |
| Failsafe Watchdog            | 200 ms signal-loss auto-disarm cutoff          |
| All-Up Flying Weight (AUW)   | 32.2 grams (including battery & airframe)      |
| Thrust-to-Weight Ratio       | 1.86 : 1 (Snappy attitude recovery)            |
| Total Build Cost per Node    | ₹2,224 (~$24.80 USD)                           |
+------------------------------+------------------------------------------------+
```

```
+-------------------------------------------------------------------------------+
|                     Dual-Core FreeRTOS Architecture                           |
+---------------------------------------+---------------------------------------+
|                CORE 0                 |                CORE 1                 |
|       (Wireless, Comms, & CLI)        |     (Deterministic Flight Physics)    |
+---------------------------------------+---------------------------------------+
| • Wi-Fi SoftAP HTTP & WebSocket Server| • 500 Hz Deterministic Timer (2000 µs)|
| • ESP-NOW Swarm Mesh Radio Rx & Tx    | • 14-Byte Contiguous I2C IMU Burst    |
| • CRC16 Packet Validation Engine      | • Complementary Attitude Filter       |
| • Swarm Peer Discovery & Table Sync   | • Cascaded Angle + Rate PID Loops     |
| • 200 ms Signal-Loss Watchdog         | • Dynamic Attitude Anti-Saturation    |
| • Interactive USB-C Serial Debug CLI  | • Quad-X Motor Torque Mixer           |
| • ADC Battery Moving-Average Filter   | • 20 kHz Hardware LEDC PWM Update     |
+---------------------------------------+---------------------------------------+
                    \                                       /
                     +--------> FreeRTOS Shared Memory <----+
```

---

## Complete Bill of Materials (BOM) & Budget

Every part on this list is readily available from standard hobby robotics retailers, Amazon, or electronics distributors:

| Component | Qty | Spec / Footprint | Unit Cost (₹) | Total (₹) | Total ($ USD) | Function / Notes |
| :--- | :---: | :--- | :---: | :---: | :---: | :--- |
| **Seeed XIAO ESP32-S3** | 1 | Dual LX7 240MHz, 2.4GHz RF | ₹850 | ₹850 | $9.50 | Flight controller MCU & mesh radio |
| **MPU9250 IMU Breakout** | 1 | 9-axis (Accel + Gyro) I2C | ₹420 | ₹420 | $4.70 | Fast attitude tracking at 400 kHz |
| **720 Coreless Brushed Motors** | 4 | 7mm × 20mm (2x CW, 2x CCW)| ₹75 | ₹300 | $3.35 | 50,000 RPM high-thrust micro motors |
| **55mm Micro Propellers** | 4 | 2x CW, 2x CCW (0.8mm bore) | ₹20 | ₹80 | $0.90 | High static thrust micro props |
| **AO3400A N-Channel MOSFETs** | 4 | SOT-23 ($V_{ds}=30\text{V}, R_{ds}<30\text{m}\Omega$) | ₹12 | ₹48 | $0.55 | 3.3V logic-level motor switches |
| **1N5819 Schottky Diodes** | 4 | DO-41 / SOD-123 ($V_f \approx 0.45\text{V}$)| ₹4 | ₹16 | $0.18 | Inductive back-EMF flyback protection |
| **Passives & Filter Caps** | 1 set| 470µF 10V electrolytic + 100nF| ₹60 | ₹60 | $0.65 | Bulk decoupling & motor RF bypass |
| **1S 3.7V 380mAh LiPo** | 1 | 25C discharge, JST-DS 1.25mm | ₹320 | ₹320 | $3.60 | ~4.5 minute hover flight time (~10.5g) |
| **3D-Printed Unibody Frame**| 1 | PLA / PLA+ (7.4g) | ₹80 | ₹80 | $0.90 | Lightweight 3D printed unibody airframe |
| **Wiring, JST & Perfboard** | 1 set| 30 AWG silicone + 25×25mm perf| ₹50 | ₹50 | $0.55 | Power rail & motor harness |
| **TOTAL PER FLIGHT NODE** | — | — | — | **₹2,224** | **$24.88** | **Full working autonomous drone** |

---

## Hardware Schematic & Wiring Blueprint

```
                     +1S LiPo Positive Rail (3.0V - 4.2V)
                                   |
         +-------------------------+-------------------------+
         |                         |                         |
     +---+---+                 +---+---+                 +---+---+
     | 470uF | Bulk            | 10uF  | MCU             | 100kΩ | Voltage Divider
     | LowESR| Storage         | Cer   | Bypass          | (1%)  | High Side
     +---+---+                 +---+---+                 +---+---+
         |                         |                         |
        GND                       GND                        +-----> XIAO Pin D0 (ADC)
         |                         |                         |
         |                         |                     +---+---+
         |                         |                     | 100kΩ | Voltage Divider
         |                         |                     | (1%)  | Low Side
         |                         |                     +---+---+
         |                         |                         |
         |                         |                        GND
         |                         |
         |       +-----------------+-----------------+
         |       |                                   |
         |   +---+---+                           +---+---+
         |   | Motor | 720 Coreless              | Cathode 1N5819
         |   |  (+)  | Brushed Motor             |  [|]  | Schottky Diode
         |   |       |                           |  / \  | (Clamps flyback)
         |   | Motor | <====== [100nF Cap] =====>| Anode |
         |   |  (-)  |   (Shunts RF noise)       +---+---+
         |   +---+---+                               |
         |       |                                   |
         |       +-----------------+-----------------+
         |                         |
         |                         | Drain (Pin 3)
         |                    +----+----+
         |    ESP32-S3        |         | AO3400A N-Channel MOSFET
         |    PWM GPIO --[100Ω]-- Gate  | (SOT-23 Package)
         |                    | (Pin 1) | Vds=30V, Id=5.7A, Rds<30mΩ
         |                    |         |
         |                    +--[10kΩ]-+ Source (Pin 2)
         |                         |         |
         |                        GND        |
         |                                   |
         +-----------------------------------+
                           |
                   POWER STAR GROUND
             (Direct to LiPo Battery Negative)
```

### Driver Circuit Design & MOSFET Selection
1. **AO3400A Logic-Level Gate Drive**: Directly driven from 3.3V GPIO through a $100\ \Omega$ series damping resistor to suppress high-frequency LC gate ringing ($C_{iss} \approx 650\text{ pF}$).
2. **10 kΩ Gate Pull-Down**: Connected directly from Gate to GND. During microcontroller bootloader initialization or flashing, GPIO pins float in high-impedance mode. Without this resistor, gate capacitance charges randomly and spins the motors uncontrollably on power-up!
3. **1N5819 Schottky Flyback Diodes**: Brushed motors are inductive coils. When the MOSFET turns off, the collapsing magnetic field creates a voltage spike:
   $$V_{\text{spike}} = -L \frac{di}{dt}$$
   Without the diode, this spike easily exceeds 40V, instantly punching through the MOSFET dielectric. The Schottky diode clamps the drain safely to $V_{bat} + 0.45\text{ V}$.
4. **470 µF Bulk Low-ESR Capacitor**: Placed directly across the battery rail adjacent to the MOSFET sources. When all four 720 motors punch out at full throttle, transient current steps exceed 7.5A. The bulk capacitor buffers these steps, preventing ESP32 brownout resets.

### EMI Mitigation: Twisted Pairs & Star Grounding
* **Twisted Motor Wiring**: Motor leads are twisted together with $\ge 4\text{ turns/cm}$. Equal and opposite currents cancel radiated magnetic fields.
* **100nF Motor Tab Capacitors**: Solder a 100nF 0805 or ceramic disc capacitor directly across each motor's terminal tabs to kill brush commutation noise before it reaches the wiring harness.
* **Dedicated Power Star Ground**: The high-current return wires from the four MOSFET sources and the 470µF capacitor meet at a single heavy-gauge copper node tied directly to the battery ground. **Never** route motor return currents through the delicate ground traces of the MPU9250 or XIAO board!

### XIAO ESP32-S3 Pin Mapping

| Board Pin | ESP32-S3 GPIO | Subsystem / Function | Hardware Connection |
| :--- | :--- | :--- | :--- |
| **D4** | `GPIO5` | **I2C SDA** | Fast-Mode 400 kHz data to MPU9250 (4.7kΩ pull-up) |
| **D5** | `GPIO6` | **I2C SCL** | Fast-Mode 400 kHz clock to MPU9250 (4.7kΩ pull-up) |
| **D2** | `GPIO4` | **Motor 1 (FL)** | Front-Left (CW) -> 100Ω -> AO3400A Gate |
| **D0** | `GPIO1` | **Motor 2 (FR)** | Front-Right (CCW) -> 100Ω -> AO3400A Gate |
| **D1** | `GPIO2` | **Motor 3 (RR)** | Rear-Right (CW) -> 100Ω -> AO3400A Gate |
| **D3** | `GPIO3` | **Motor 4 (RL)** | Rear-Left (CCW) -> 100Ω -> AO3400A Gate |
| **D8** | `GPIO8` | **Battery ADC** | 2:1 divider (100kΩ / 100kΩ) sensing 1S LiPo |
| **LED** | `GPIO21` | **Status LED** | Onboard active-LOW diagnostic indicator |
| **3V3** | — | **3.3V Regulated** | Clean logic power to MPU9250 IMU |
| **GND** | — | **Logic Ground** | Tied to Power Star Ground at battery pad |

---

## Airframe 3D Printing & Fabrication Guide

The unibody airframe is optimized for rapid FDM 3D printing on any entry-level printer (Ender 3, Bambu Lab, Prusa, etc.):

```
                  FRONT (Nose)
                       ^
                       |
         M1 (FL, CW)   |   M2 (FR, CCW)
            \          |          /
             \         |         /
              \   +----+----+   /
               \  | XIAO S3 |  /
      <--------+--| MPU9250 |--+--------> (Roll Right: +)
               /  +----+----+  \
              /        |        \
             /         |         \
         M4 (RL, CCW)  |   M3 (RR, CW)
                       |
                  REAR (Tail)
```

### Slicer Configuration Profile
* **Material**: Standard PLA or PLA+ (e.g. eSUN PLA+).
* **Layer Height**: `0.16mm` or `0.20mm`.
* **Perimeters / Wall Loops**: `5` (Ensure the motor arms are **100% solid plastic** without hollow infill to eliminate vibration resonance).
* **Infill**: `100%` on arms, `25% Gyroid` on central electronics tray.
* **Critical Setting — Horizontal Hole Expansion**: Set to **`+0.12mm`**.  
  *(This compensates for molten plastic shrinkage, guaranteeing the motor pods come off the bed at exactly 7.02mm for a snug, snap-free friction fit with 7.00mm motors).*
* **Support**: None required (designed with 45° self-supporting overhangs).
* **Print Time**: ~45 minutes.

### Weight Budget Breakdown

| Subsystem Component | Target Weight | Actual Measured Weight |
| :--- | :---: | :---: |
| 3D Printed Unibody Frame (PLA) | 7.5 g | **7.4 g** |
| 4 × 720 Coreless Brushed Motors | 18.0 g | **18.2 g** |
| 4 × 55mm Propellers | 1.2 g | **1.1 g** |
| Seeed Studio XIAO ESP32-S3 | 2.1 g | **2.1 g** |
| MPU9250 IMU Breakout Board | 1.2 g | **1.2 g** |
| MOSFET Driver Board + Passives | 2.0 g | **1.8 g** |
| 1S 380mAh LiPo Battery | 10.5 g | **10.4 g** |
| Wiring Harness & Solder | 1.5 g | **1.4 g** |
| **ALL-UP FLYING WEIGHT (AUW)** | **44.0 g (Max)** | **43.6 g (Flight Ready)** |

*Static thrust generated: 4 × 38g = **152g total thrust**.*  
*Thrust-to-weight ratio: $\frac{152\text{g}}{43.6\text{g}} = \mathbf{3.48 : 1}$ (plenty of power for sharp recovery).*

---

## Firmware Architecture

### Dual-Core FreeRTOS Task Allocation
1. **Core 1 — Flight Loop Task (`Priority 24`, Highest)**:
   Runs strictly every **2000 µs (500 Hz)**. Does not yield, does not perform dynamic memory allocations, and never blocks on Wi-Fi or serial operations. Executes IMU burst reads, attitude estimation, cascaded PID loops, dynamic anti-saturation mixer, and LEDC updates in under **420 µs**, leaving $>75\%$ deterministic CPU headroom.
2. **Core 0 — Comms & Background Task (`Priority 5`)**:
   Runs non-blocking Wi-Fi SoftAP HTTP/WebSocket services, handles ESP-NOW packet reception and heartbeat broadcasting, reads battery ADC moving averages, drives diagnostic LED blinks, and services the interactive USB Serial CLI.

### Dynamic Priority Anti-Saturation Mixer
A classic pitfall in micro-quadcopters is motor saturation. When high throttle coincides with large roll, pitch, or yaw corrections, calculated motor outputs can exceed 1023 (100% duty). If outputs were simply clamped, differential torque is lost, causing the drone to flip and crash.

ResQmesh implements **Dynamic Attitude Priority Anti-Saturation**:
$$M_1 (\text{FL}) = \text{Throttle} + \text{Roll} - \text{Pitch} - \text{Yaw}$$
$$M_2 (\text{FR}) = \text{Throttle} - \text{Roll} - \text{Pitch} + \text{Yaw}$$
$$M_3 (\text{RR}) = \text{Throttle} - \text{Roll} + \text{Pitch} - \text{Yaw}$$
$$M_4 (\text{RL}) = \text{Throttle} + \text{Roll} + \text{Pitch} + \text{Yaw}$$

If $\max(M_1, M_2, M_3, M_4) > 1023$:
$$\text{Excess} = \max(M_1, M_2, M_3, M_4) - 1023$$
$$\text{Throttle} \leftarrow \text{Throttle} - \text{Excess}$$
All four motors are lowered equally. The drone sacrifices a few percent of climb rate while preserving 100% of its roll, pitch, and yaw stabilization torque!

### Cascaded Dual-Loop PID Controller
* **Outer Angle Loop (P-Loop)**: Converts stick tilt demands ($\pm30^\circ$) into desired angular rates ($\text{deg/s}$).
  $$\text{Target Rate} = K_{p,\text{angle}} \cdot (\text{Target Angle} - \text{Estimated Angle})$$
* **Inner Rate Loop (PID-Loop)**: Compares desired angular rate with instantaneous gyro rates from the MPU9250.
  $$\text{Torque} = K_p \cdot e + K_i \int e \, dt + K_d \frac{d(e_{\text{filtered}})}{dt}$$
* **Derivative Low-Pass Filter**: D-term is filtered through a 1st-order low-pass filter ($\tau = 0.005\text{ s}$, cutoff $\approx 31.8\text{ Hz}$) to eliminate high-frequency motor vibration buzz.
* **Anti-Windup Clamping**: Integrators are strictly bounded to prevent saturation overshoot during takeoff or ground strikes.

---

## The Swarm Mesh Network: ESP-NOW + Phone SoftAP

ResQmesh solves the fundamental connectivity dilemma by running **concurrent Dual-Mode Radio**:

```
                  2.4 GHz Wi-Fi Hotspot (Channel 1)
 [Smartphone] <=====================================> [Drone #1 (Swarm Leader)]
 Touch UI       • 40 Hz Stick Control                 • SoftAP + WebSockets Server
 Browser        • Live HUD Telemetry                  • 500 Hz Real-Time Flight Loop
 (192.168.4.1)  • MESH: "2 NODES" (Peer Status)      • Swarm Peer Table
                                                               |
                                                               | Concurrent 2.4 GHz ESP-NOW
                                                               | Broadcasts (Channel 1)
                                                               v
                              +--------------------------------+-------------------------------+
                              |                                                                |
                              v                                                                v
                  [Drone #2 (Follower)]                                            [Drone #3 (Follower)]
                  • Pure ESP-NOW Mode 0                                            • Pure ESP-NOW Mode 0
                  • Executes Synchronized Sticks                                   • Executes Synchronized Sticks
                  • 10 Hz Heartbeat & Telemetry                                    • 10 Hz Heartbeat & Telemetry
```

### Concurrent Dual-Mode Radio Operation
1. The 2.4 GHz radio on the Seeed Studio XIAO ESP32-S3 is configured on **Channel 1** for both Wi-Fi SoftAP and ESP-NOW (`WIFI_AP_CHANNEL == ESPNOW_WIFI_CHANNEL`).
2. Drone #1 acts as the **Swarm Leader / Bridge**:
   - It hosts the local Wi-Fi hotspot (`ResQmesh-Drone`) and embedded HTTP/WebSocket server.
   - It concurrently initializes `esp_now_init()` and registers the broadcast peer (`FF:FF:FF:FF:FF:FF`).
3. As the pilot moves the virtual joysticks or slides the arm bar on their phone, Drone #1 applies the flight inputs locally and **simultaneously re-broadcasts a `SwarmControlPacket` with CRC16 validation over ESP-NOW**.
4. Follower drones listening on ESP-NOW receive the packet in $<5\text{ ms}$, executing the exact same maneuver in synchronized formation.
5. All follower drones broadcast a 10 Hz `SwarmHeartbeatPacket`. Drone #1 receives these, populates its `SwarmPeerTable`, and reports `"peers": N` over WebSocket, displaying `MESH: 2 NODES` directly on your phone screen!

### Piloting with Any Smartphone (Zero-Install Touch UI)
No app store downloads, no drivers, no internet connection required:
1. Turn on the drone. Connect phone Wi-Fi to **`ResQmesh-Drone`** (Password: **`12345678`**).
2. Open Safari (iOS) or Chrome (Android) to **`http://192.168.4.1`**.
3. **Left Joystick**: Throttle ($0-100\%$) and Yaw (rudder, $\pm200^\circ/\text{s}$).
4. **Right Joystick**: Pitch (nose forward/back) and Roll (bank left/right).
5. **Slide-to-Arm**: Built-in safety interlock requires throttle at $0\%$ to arm.
6. **Red KILL Button**: Emergency cut zeroes all four motor duties in $<10\text{ ms}$.
7. **Trims & Max Throttle Limit**: Set a 60% throttle ceiling for safe indoor flying.

---

## Step-by-Step Replication Guide

### Step 1: 3D Printing the Airframe
1. Slice the unibody frame STL using the settings above:
   * 5 wall perimeters (100% solid arms).
   * Set **Horizontal Hole Expansion to `+0.12mm`**.
2. Print in PLA or PLA+.
3. Clean any stringing from the central electronics bay.
4. Test-fit one 720 coreless motor into a motor pod. It should insert with firm thumb pressure and stay securely locked without cracking the plastic.

### Step 2: Soldering the MOSFET Driver Board
Cut a 25mm × 25mm section of standard 2.54mm perfboard:
1. **Mount the 4 × AO3400A MOSFETs**: Use the dead-bug technique or orient them across adjacent pads.
2. **Solder the 10 kΩ Gate Pull-Down Resistors**: Connect one between each MOSFET Gate (Pin 1) and Source (Pin 2).
3. **Solder the 100 Ω Gate Series Resistors**: Connect one to each Gate pad, leaving the other leg free to receive the signal wire from the ESP32.
4. **Tie all 4 Sources together**: Connect all Source pins (Pin 2) with solid copper bus wire. This forms the high-current Power Ground.
5. **Solder the 4 × 1N5819 Schottky Diodes**: Connect each diode's **Cathode** (striped end) to the battery positive rail ($V_{bat}$) and **Anode** to the MOSFET Drain (Pin 3).
6. **Install the 470 µF Bulk Capacitor**: Solder directly across the $V_{bat}$ and Power Ground rails on the board.

### Step 3: Motors, Decoupling & Harness Assembly
1. Press-fit the 4 × 720 coreless motors into the frame:
   * **Front-Left (M1)**: Clockwise (CW) motor (Red/Blue wires).
   * **Front-Right (M2)**: Counter-Clockwise (CCW) motor (Black/White wires).
   * **Rear-Right (M3)**: Clockwise (CW) motor (Red/Blue wires).
   * **Rear-Left (M4)**: Counter-Clockwise (CCW) motor (Black/White wires).
2. Solder a **100nF ceramic capacitor** directly across each motor's solder tabs.
3. Tightly twist each motor's wire pair ($\ge 4\text{ turns/cm}$) and route them along the frame arms into the center tray.
4. Connect each motor (+) to $V_{bat}$ and motor (-) to the respective MOSFET Drain.
5. Solder the Seeed Studio XIAO ESP32-S3 and MPU9250 boards into the top deck:
   * Connect XIAO 3.3V and GND to the MPU9250 power pins.
   * Connect `GPIO5` to MPU9250 `SDA` and `GPIO6` to `SCL`.
   * Connect `GPIO4` (M1), `GPIO1` (M2), `GPIO2` (M3), `GPIO3` (M4) to the respective 100Ω gate resistors.

### Step 4: Toolchain Setup & Firmware Flashing

#### Option A: 1-Click Automated Flasher (Windows)
Plug your XIAO ESP32-S3 into your PC via USB-C and run:
```cmd
.\code\ESP32-DRONE\flash_drone.bat
```
The script auto-detects your COM port, compiles the sketch with Arduino CLI, and flashes the firmware.

#### Option B: PlatformIO
```bash
cd code/ESP32-DRONE
pio run -t upload -t monitor
```

#### Option C: ESP-IDF
```bash
cd code/ESP32-DRONE
idf.py set-target esp32s3
idf.py build
idf.py -p COM_PORT flash monitor
```

### Step 5: Automated Testing & Verification
Before connecting batteries, run the automated host unit tests on your PC:
```powershell
cd code/ESP32-DRONE/tests
.\run_tests.bat
```
*(Confirms all 73 algorithmic assertions pass: PID anti-windup, complementary filter convergence, anti-saturation mixer, CRC16 validation, battery alarms, and dual-mode mesh packet relay).*

To run the full 500 Hz flight physics simulation:
```powershell
.\run_simulation.bat
```

### Step 6: Safe Commissioning Checklist (Props OFF!)

> [!CAUTION]
> **NEVER INSTALL PROPELLERS UNTIL ALL 12 STEPS BELOW ARE VERIFIED.**

- [ ] **Step 1: REMOVE ALL PROPELLERS.**
- [ ] **Step 2: Check Power Rails for Shorts.** Use a multimeter in continuity mode across $V_{bat}$ and GND. Ensure resistance is $>1\text{ k}\Omega$ (no solder bridges).
- [ ] **Step 3: Connect USB-C.** Open serial monitor at **115200 baud**. Confirm `status` prints healthy boot messages and IMU calibration succeeds.
- [ ] **Step 4: Verify Motor Spin Directions**:
  - In serial console, type `test_motor 1 5` -> M1 (Front-Left) must spin **Clockwise (CW)**.
  - Type `test_motor 2 5` -> M2 (Front-Right) must spin **Counter-Clockwise (CCW)**.
  - Type `test_motor 3 5` -> M3 (Rear-Right) must spin **Clockwise (CW)**.
  - Type `test_motor 4 5` -> M4 (Rear-Left) must spin **Counter-Clockwise (CCW)**.
- [ ] **Step 5: Verify Attitude Estimation**:
  - Type `attitude` in console.
  - Tilt drone 20° right wing down -> Roll must read $\approx +20.0^\circ$.
  - Tilt drone 20° nose up -> Pitch must read $\approx +20.0^\circ$.
- [ ] **Step 6: Verify Gyroscopic Counter-Torque**:
  - Arm the drone at 15% throttle.
  - Physically tilt the drone right wing down with your fingers.
  - The right motors (M2, M3) must automatically speed up and left motors (M1, M4) must slow down to resist the tilt.
- [ ] **Step 7: Verify Failsafe Cutoff**:
  - With the drone armed, disconnect the Wi-Fi or close the browser controller.
  - All 4 motors must **instantly stop within 200 milliseconds**.

### Step 7: Tethered Hover & Initial Flight
1. Install the 55mm propellers:
   * **Orange / Black CW Props** on M1 (FL) and M3 (RR).
   * **Black CCW Props** on M2 (FR) and M4 (RL).
2. Attach a loose thread or elastic safety tether from the frame to a weighted object on the floor.
3. Connect your phone to `ResQmesh-Drone`, open `http://192.168.4.1`, and gently advance throttle past 35%.
4. Observe the aircraft enter a stable, level hover. Trim roll and pitch using the quick trim buttons on the mobile screen.

---

## Desktop Ground Control Station & Tools

ResQmesh includes a high-performance desktop Ground Control Station (GCS) built with WebGL and HTML5:

* **Launch Desktop GCS**: Double-click [open_gui.bat](code/ESP32-DRONE/open_gui.bat)
  * Real-time 3D attitude horizon and quadcopter wireframe visualizer.
  * 4-channel real-time motor duty oscilloscopes.
  * Live IMU accelerometer & gyroscope spectral FFT graphs.
  * Tactical Swarm Radar displaying multi-node coordinates and peer health.
* **Launch Mobile Preview**: Double-click [open_mobile_preview.bat](code/ESP32-DRONE/open_mobile_preview.bat)
  * Test and simulate the smartphone touch controller directly in your desktop browser.

---

## Repository Layout

```
ResQmesh/
├── README.md                      # Complete project documentation & replication guide
├── mkdocs.yml                     # Documentation site generator config
├── docs/                          # Weekly engineering build logs (Week 1 to Week 10)
│   ├── index.md                   # Project overview and mission roadmap
│   ├── week-01.md                 # The ₹3,000 Swarm Gamble
│   ├── week-02.md                 # 3D Printing, Snapped Arms & Slicer Shrinkage
│   ├── week-03.md                 # Counting Pennies & Component Selection
│   ├── week-06.md                 # Hand-Wired Micro Flight Controller Blueprint
│   └── week-10.md                 # Dual-Core FreeRTOS Flight Loop & First Flight
├── code/
│   ├── README.md                  # Firmware and controller subsystem summary
│   ├── ESP32-DRONE/               # Primary Flight Controller & Swarm Firmware
│   │   ├── firmware/              # Arduino CLI sketch source & configurations
│   │   ├── main/                  # ESP-IDF / PlatformIO modular C++ source
│   │   │   ├── config.h           # Central pinout, timing & PID configuration
│   │   │   ├── flight_controller.cpp # 500 Hz deterministic FreeRTOS loop
│   │   │   ├── imu.cpp            # MPU9250 Fast-I2C driver & zero-bias calibration
│   │   │   ├── attitude.cpp       # 500 Hz complementary filter
│   │   │   ├── pid.cpp            # Cascaded outer angle + inner rate PID loops
│   │   │   ├── mixer.cpp          # Quad-X dynamic anti-saturation mixer
│   │   │   ├── motors.cpp         # 20 kHz LEDC ultrasonic PWM driver
│   │   │   ├── receiver.cpp       # Dual-mode Wi-Fi SoftAP + ESP-NOW Swarm Bridge
│   │   │   ├── swarm.h            # Multi-agent mesh framing & peer table
│   │   │   ├── web_ui.h           # PROGMEM-stored smartphone touch web app
│   │   │   └── cli.cpp            # Interactive USB-C serial debugging shell
│   │   ├── gui/                   # Desktop GCS & mobile web applications
│   │   │   ├── index.html         # 3D Attitude visualizer & Swarm Radar GCS
│   │   │   ├── mobile.html        # Smartphone touch flight controller web app
│   │   │   └── mobile_preview.html# Standalone browser simulator
│   │   ├── tests/                 # Automated native host test suite
│   │   │   ├── run_tests.bat      # 1-click test runner (73 unit test assertions)
│   │   │   └── run_simulation.bat # 500 Hz end-to-end flight physics simulator
│   │   └── flash_drone.bat        # 1-click automated USB-C flasher script
│   └── ESP32-CONTROLLER/          # Physical RC Transmitter / Ground Gateway
│       └── main/main.cpp          # ESP-NOW 50 Hz handheld transmitter firmware
└── cad/                           # 3D-printable airframe CAD models (STL, STEP)
```

---

## License & Acknowledgments

This project is licensed under the **MIT License** — feel free to modify, replicate, and deploy for educational, research, or commercial swarm applications.

**Developed as part of the Builder-in-Residence Program.**  
*Lead Engineer: Jayasurya Jayakumar*
