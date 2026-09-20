# Week 10: Breathing Life into the Swarm: Dual-Core FreeRTOS Firmware

Ten weeks ago, this project was just numbers on a whiteboard: a crazy idea to build a collaborative swarm drone for under ₹3,000. 

This week, all the threads converged. We assembled the 7.4g unibody frame, mounted the custom MOSFET driver board, wired the Seeed Studio XIAO ESP32-S3, and flashed our ground-up C++ flight control and swarm networking firmware.

---

## Why We Wrote Our Own Flight Stack

Early on, several people asked us: *"Why not just flash Betaflight or ArduPilot and call it a day?"*

The answer comes down to what a swarm actually is. Off-the-shelf flight stacks are designed for single acrobatic FPV quads or heavy GPS autonomous drones running on STM32 microcontrollers. They expect dedicated radio receivers (CRSF/ELRS) feeding UART ports, and their monolithic scheduling loops don't play nicely with ESP32 Wi-Fi/RF radios.

To make a true swarm drone, we needed:
1. **Direct Dual-Core Control:** The ESP32-S3 has two 240 MHz Xtensa LX7 cores. We wanted zero jitter on our flight stabilization loop, which meant dedicating an entire core exclusively to flight physics.
2. **Native ESP-NOW Swarm Protocol:** We needed lightweight, peer-to-peer packet broadcasting between drones at sub-5ms latency, without the overhead of heavy IP/TCP network stacks.
3. **Tiny Binary Footprint:** Our entire compiled firmware flashes in under 4 seconds over USB-C, leaving huge flash headroom for future sensor drivers and autonomous swarm behaviors.

```
+-------------------------------------------------------------------------+
|                       ESP32-S3 Dual-Core Architecture                   |
+------------------------------------+------------------------------------+
|               CORE 0               |               CORE 1               |
|       (Asynchronous Swarm & IO)    |     (Deterministic 500 Hz Flight)  |
+------------------------------------+------------------------------------+
| • ESP-NOW Swarm Mesh Radio         | • 500 Hz Timer Interrupt (2000 µs) |
| • CRC16 Packet Validation          | • MPU9250 Fast I2C Burst Read      |
| • 200 ms Failsafe Watchdog         | • Complementary Attitude Filter    |
| • USB Interactive Serial CLI       | • Cascaded Angle + Rate PID Loop   |
| • Battery Voltage ADC Monitoring   | • Anti-Saturation Quad-X Mixer     |
| • Telemetry Logging                | • 20 kHz Ultrasonic LEDC PWM Out   |
+------------------------------------+------------------------------------+
                   \                                      /
                    +----> FreeRTOS Shared Memory <------+
```

---

## The 500 Hz Deterministic Loop (Core 1)

On Core 1, our flight loop runs strictly every **2,000 microseconds** ($2.0\text{ ms}$):

1. **IMU Burst Read (14 Bytes):** Grabs 3-axis gyro and 3-axis accelerometer registers in a single contiguous I2C transaction at 400 kHz (~35 µs).
2. **Attitude Fusion:** Merges gyro rates and accelerometer angles through our complementary filter to track vehicle Roll and Pitch.
3. **Cascaded Dual-Loop PID:**
   - **Outer Angle Loop:** Takes pilot/swarm tilt demands ($\pm25^\circ$) and calculates desired angular rates ($\text{deg/s}$).
   - **Inner Rate Loop:** Computes instantaneous motor torque corrections, filtered through a low-pass D-filter ($\tau = 0.005\text{s}$) to kill vibration noise.
4. **Anti-Saturation Mixer:** Combines Throttle, Roll, Pitch, and Yaw into 4 motor duties. If full throttle tries to clip any motor past 1023, it dynamically scales down collective throttle, ensuring the drone never loses attitude authority in an aggressive maneuver.
5. **Hardware PWM Output:** Writes 10-bit duty cycles to the 20 kHz LEDC timers.

We measured our total loop execution time on an oscilloscope GPIO toggle: **$420\ \mu\text{s}$ out of our $2000\ \mu\text{s}$ budget.** That leaves over **75% idle CPU headroom** on Core 1!


---

## Swarm Comms & The 200 ms Dead-Man Switch (Core 0)

While Core 1 is hyper-focused on balancing the drone, Core 0 handles the outside world via **ESP-NOW**:

- **Swarm Packet Framing:** Every packet is packed into a compact binary structure containing a target Node ID (`0x01` through `0xFF`), a rolling packet sequence number, 4 joystick axes, and a **CRC16 checksum**. If RF noise corrupts a single bit in the air, the checksum fails and the corrupted packet is discarded in microseconds.
- **The 200 ms Failsafe Watchdog:** If an individual drone drops out of radio range or an interfering signal blocks communication for more than 200 milliseconds, an independent watchdog state machine trips. It instantly overrides the mixer, cuts all four motor duties to `0`, and transitions the drone to `DISARMED`. No rogue flyaways, no runaway drones.
- **Interactive USB CLI:** You can plug the drone into a laptop, open a serial terminal at 115200 baud, and run live diagnostic commands (`status`, `imu`, `motors`, `test_motor`) without interrupting the 500 Hz flight loop on the other core.

---

## The Hand Test: It Fights Back!

With everything flashed, we held the assembled quadcopter loosely between two fingers, armed the flight controller, and brought the throttle up to 25%.

The four 55mm propellers spun into a quiet, smooth blur. 

Then we tried to tilt the drone forward with our fingers. 

Instantly, the front two motors roared to life with increased RPM while the rear two backed off, delivering a distinct, firm gyroscopic counter-torque right into our fingertips. When we tilted it left, the left motors surged to push it level. It felt alive—actively resisting our hand, fighting with everything it had to hold a perfectly flat, level plane in 3D space.

```
Pitch Nose Down by Hand:  [ Front Motors Spool Up! Rear Motors Cut Back! ]
Roll Left Wing by Hand:   [ Left Motors Surge! Right Motors Back Off! ]
Release to Neutral:       [ All Four Motors Settle into Smooth Equilibrium ]
```

https://github.com/user-attachments/assets/9f020abd-1319-4267-80d9-c85300ebb805



Ten weeks of late nights, broken prints, courier delays, schematic revisions, and microscopic soldering had brought us to this moment. The hardware works, the electrical design is $>98\%$ efficient, the flight loop is rock-solid at 500 Hz, and the entire drone weighs **32.2 grams** on a total build cost of **₹2,224**.

---

## What Comes Next

With the core flight platform fully operational, the foundation for **ResQmesh** is built:

1. **Indoor Tethered Hover Flights:** Fine-tuning PID gains ($K_p, K_i, K_d$) in free air with safety tethers.
2. **Multi-Node Swarm Pairing:** Flashing three identical units and broadcasting synchronized throttle and formation vectors via ESP-NOW.
3. **Sensor Modules:** Snapping on the VL53L1X Time-of-Flight sensor for ground-distance altitude hold and obstacle avoidance.

The swarm has taken its first breath. Now, it's time to fly.
