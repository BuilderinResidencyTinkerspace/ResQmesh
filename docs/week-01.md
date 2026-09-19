# Week 1: The ₹3,000 Swarm Gamble

Every drone swarm project we’ve ever seen starts with a jaw-dropping budget: tens of thousands of dollars, proprietary radio links, and drones so expensive that watching one clip a tree branch feels like watching a stack of cash catch fire. 

We wanted to flip that script completely. What if a swarm wasn't made of precious, monolithic aircraft, but of disposable, dirt-cheap micro-drones that work together like ants? If one runs out of juice or crashes into a wall, the mission shouldn’t fail—the rest of the swarm should just adapt and keep moving.

That was the core question that kicked off **ResQmesh**.

---

## Setting an Impossible Constraint

We started on the whiteboard with one non-negotiable rule: **every single flight node has to cost under ₹3,000 (~$35 USD).** 

```
Commercial Swarm Unit:  [ $2,000+ ]  --> Lose one = Catastrophe
ResQmesh Target Unit:   [ < ₹3,000 ]  --> Lose one = Keep flying
```

That single financial ceiling immediately killed dozens of comfortable off-the-shelf options:
- Brushless motors and 4-in-1 ESCs? Out. Too heavy for a micro frame, and an ESC stack alone would eat our entire budget.
- Full-sized STM32 flight controllers? Out. Expensive and bulky.
- Commercial GPS modules? Out. They don't work reliably indoors anyway, and good ones blow past our price ceiling instantly.

Instead, we decided to build around the **Seeed Studio XIAO ESP32-S3**. It packs a dual-core Xtensa LX7 running at 240 MHz, built-in 2.4 GHz Wi-Fi and BLE, native USB, and enough GPIOs for an I2C bus and 4 PWM channels—all on a postage-stamp-sized PCB for a fraction of the cost of dedicated flight boards.

---

## Brainstorming Where This Actually Matters

We didn’t want to build a swarm just to watch blinking LEDs in an auditorium (though that’s cool too). We wanted a platform with real, dirty, practical utility. Over a marathon whiteboarding session, we narrowed down six mission profiles where expendable micro-swarms solve problems single big drones can’t touch:

1. **Search & Rescue in Tight Rubble:** After a structural collapse, heavy drones can't squeeze through broken concrete or navigate dust-filled hallways. Micro quadcopters can slip into gaps, sniff for survivors, and act as short-range audio/video beacons.
2. **Precision Canopy & Microclimate Mapping:** Instead of taking one reading at a time with a handheld probe, imagine dropping ten tiny nodes across a greenhouse or orchard to measure humidity, temperature, and light gradients simultaneously.
3. **Off-Grid Aerial Mesh Repeaters:** Ground radios die when hills or thick concrete walls get in the way. Two or three hovering ESP-NOW nodes can form a dynamic airborne hop-line, relaying emergency sensor data over obstacles.
4. **GPS-Denied Warehouse & Duct Inspection:** Flying where GPS signals don't penetrate—ventilation shafts, crawl spaces, and factory rafters.
5. **Open Swarm Robotics Testbed:** Most university labs can't afford a fleet of 20 research drones for testing Reynolds Boids flocking or multi-agent reinforcement learning (MARL). A sub-₹3,000 node changes that equation.
6. **Synchronized Miniature Light Choreography:** Coordinated indoor formation flights using high-brightness addressable RGBs without needing a football stadium.

---

## The First Architecture Sketch

By the end of the week, our rough system architecture was locked in:

| Subsystem | Selected Component | Why We Picked It |
| :--- | :--- | :--- |
| **Brain / MCU** | Seeed Studio XIAO ESP32-S3 | Dual-core 240 MHz compute + native ESP-NOW radio on a tiny board |
| **IMU** | InvenSense MPU9250 / MPU6050 | Fast 400 kHz I2C attitude tracking with well-documented register maps |
| **Motors** | 720 Coreless Brushed DC (7×20mm) | ~40g thrust each on 55mm props; pennies per motor; zero ESC needed |
| **Switches** | AO3400A N-Channel MOSFETs | Logic-level gate threshold (<1.45V) driven directly by 3.3V GPIOs |
| **Battery** | 1S 3.7V LiPo (~300–450 mAh) | Keeps all-up weight under 35g to maintain a healthy >1.8:1 thrust margin |
| **Mesh Comms** | Peer-to-Peer ESP-NOW | Sub-5ms packet latency without routers or cellular towers |

---

## What Kept Us Up at Night

The biggest open question wasn’t the code—it was the physics. A brushed 720 coreless motor is cheap, but it generates nasty inductive flyback when you pulse it with high-frequency PWM. If those voltage spikes bleed into the ESP32’s power rail or the MPU9250’s I2C lines, the microcontroller will brown out, the sensor bus will hang, and the drone will tumble out of the sky.

Next week, we fire up Fusion 360 and the 3D printers to see if we can build an airframe stiff enough to survive motor vibration without blowing our weight budget.
