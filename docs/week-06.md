# Week 6: The Anatomy of a Hand-Wired Micro Flight Controller

Designing a schematic on a computer screen is clean and theoretical. Everything is a neat orthogonal line, nets connect with a click, and wires never accidentally touch each other. 

Building a physical flight controller by hand on a 25mm × 25mm perfboard slice is an entirely different beast. At this scale, stray capacitance, wire routing paths, and millimeter-long solder bridges can make the difference between a rock-solid hover and a smoky microcontroller.

With the final courier delivery scheduled for next week, we dedicated Week 6 to turning our KiCad schematic into a meticulous 3D wiring blueprint.

---

## The Geometry of a SOT-23 on Perfboard

The most challenging component in our circuit is the **AO3400A MOSFET**. It comes in an ultra-compact surface-mount **SOT-23** package—designed for machine pick-and-place on manufactured PCBs, not for human hands holding a soldering iron.

```
       SOT-23 Pinout (Top View):
              +---+---+
        Gate  | 1   3 |  Drain (To Motor Negative)
              |       |
      Source  | 2     |
              +-------+
                 |
         (To Power GND)
```

Standard perfboard has a 2.54mm (0.1-inch) hole pitch. SOT-23 pins are spaced at a minuscule 0.95mm. If you try to jam a SOT-23 flat onto standard perfboard holes, the legs don't reach the pads, and any excess solder will bridge Gate to Source.

To solve this without ordering commercial PCBs:
1. We mapped out a custom **"dead-bug / bridge" mounting technique**: bending the Drain lead upward to connect directly to the flyback diode cathode, while Source solders flat to a shared solid-copper ground bus wire running along the bottom.
2. We placed the **100Ω gate resistor** directly across the Gate pin before attaching any flexible wire, acting as a physical bridge and dampening high-frequency reflections before they travel down the signal line.

---

## Combating EMI: Twisting Wires Like Network Cables

Brushed coreless motors are essentially tiny mechanical spark generators. As the internal commutator brushes sweep across the armature segments at 50,000 RPM, they create continuous micro-arcing. That arcing broadcasts high-frequency electromagnetic interference (EMI) into the air.

If you run parallel, straight wires from your motors right past your MPU9250 sensor, those wires act as miniature antennas. They radiate RF hash straight into the 3.3V power rails and induce false spikes on the I2C Clock (`SCL`) and Data (`SDA`) lines.

```
Parallel Motor Wires:  =================  --> Radiates EMI directly into sensor lines!
Twisted Motor Pairs:   -X-X-X-X-X-X-X-X-  --> Magnetic fields cancel out! Low EMI.
```

Our layout plan standardized two physical rules:
- **Tightly Twisted Pairs:** Every motor's positive and negative power leads must be twisted together with at least 4 turns per centimeter before running inward to the central driver board. The opposing currents create equal and opposite magnetic fields that cancel each other out.
- **Physical Separation:** Motor power lines are routed along the bottom arms of the 3D-printed frame, while the MPU9250 I2C signals run along the top deck, maintaining an air gap between high-current power switching and low-voltage logic.

---

## The Complete Pin Mapping

By the end of the week, our physical wiring harness was fully mapped to the Seeed Studio XIAO ESP32-S3:

| XIAO Pin | GPIO | Function | Connection Details |
| :--- | :--- | :--- | :--- |
| **D0** | `GPIO1` | Battery Voltage ADC | 2:1 divider (100kΩ / 100kΩ) sensing 1S LiPo voltage |
| **D1** | `GPIO2` | Motor 1 PWM (Front-Left) | 20 kHz LEDC PWM output -> 100Ω gate resistor -> AO3400A Gate |
| **D2** | `GPIO3` | Motor 2 PWM (Front-Right)| 20 kHz LEDC PWM output -> 100Ω gate resistor -> AO3400A Gate |
| **D3** | `GPIO4` | Motor 3 PWM (Rear-Right) | 20 kHz LEDC PWM output -> 100Ω gate resistor -> AO3400A Gate |
| **D4** | `GPIO5` | I2C SDA | Fast-Mode 400 kHz data line to MPU9250 IMU |
| **D5** | `GPIO6` | I2C SCL | Fast-Mode 400 kHz clock line to MPU9250 IMU |
| **D6** | `GPIO43` | Motor 4 PWM (Rear-Left)  | 20 kHz LEDC PWM output -> 100Ω gate resistor -> AO3400A Gate |
| **3V3** | — | Regulated 3.3V Out | Dedicated quiet power rail to MPU9250 IMU |
| **GND** | — | Logic Ground | Star-ground tie point to battery negative terminal |
<img width="430" height="550" alt="WhatsApp Image 2026-09-20 at 9 37 58 AM" src="https://github.com/user-attachments/assets/a9a54c20-0ad5-4db5-9d83-a4f46e3e1392" />
<img width="300" height="400" alt="20260916_021730" src="https://github.com/user-attachments/assets/183089fb-b13e-42bd-89ec-64ee2dae1e5a" />


https://github.com/user-attachments/assets/fb0bdad3-6bdf-4919-a269-c35ca3fdb0d8


---

## On Deck

The blueprint is ready, the routing paths are marked, and courier tracking confirmed the components are in the local delivery van for early Monday morning.

Next week, we write and validate our entire 500 Hz flight loop on a PC simulator before flashing the actual hardware.
