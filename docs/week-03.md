# Week 3: Counting Pennies & Chasing Specs

Building a prototype on a breadboard with whatever parts you have lying around is easy. Building a repeatable, ultra-low-cost drone where every component has to hit a ruthless weight, electrical, and budget threshold is an exercise in ruthless optimization.

This week was all about component selection, datasheet digging, and placing the hardware orders for our fleet.

---

## The ₹3,000 Bill of Materials Challenge

We sat down with a spreadsheet, scouring domestic suppliers, local robotics shops, and wholesale electronics portals. Every single rupee had to justify its existence on the bill of materials.

Here’s where our numbers landed:

| Subsystem / Part | Qty | Unit Cost (₹) | Total (₹) | Why This Specific Part? |
| :--- | :---: | :---: | :---: | :--- |
| **Seeed Studio XIAO ESP32-S3** | 1 | 850 | 850 | Dual-core compute, onboard antenna, native USB-C, ultra-tiny footprint |
| **InvenSense MPU9250 IMU** | 1 | 420 | 420 | Fast 400 kHz I2C attitude tracking with 3-axis gyro + 3-axis accel |
| **720 Coreless Brushed Motors (CW/CCW pairs)** | 4 | 75 | 300 | 50,000+ RPM punch; lightweight cylindrical form factor |
| **55mm Micro Propellers (2x CW, 2x CCW)** | 4 | 20 | 80 | High static thrust matched to 7mm motor shaft |
| **AO3400A N-Channel MOSFETs (SOT-23)** | 4 | 12 | 48 | Logic-level switching ($R_{ds(\text{on})} < 30\text{ m}\Omega$ @ 3.3V gate) |
| **1N5819 Schottky Flyback Diodes** | 4 | 4 | 16 | Ultra-fast back-EMF clamping across motor inductive loads |
| **Passives (Caps, Resistors, Star-Ground Perfboard)** | 1 set | 60 | 60 | 470µF bulk low-ESR cap, 100nF decoupling, gate pulldowns |
| **1S 3.7V 380 mAh LiPo Battery** | 1 | 320 | 320 | High discharge rate (25C), lightweight (~10.5g) |
| **3D-Printed Unibody Micro-X Frame** | 1 | 80 | 80 | Raw PLA filament cost for our 7.4g optimized frame |
| **Misc (Wiring, JST connectors, silicone lead)** | 1 set | 50 | 50 | High-strand-count silicone wire for minimal resistance |
| **TOTAL BOM PER FLIGHT NODE** | — | — | **₹2,224** | **Well below our ₹3,000 ceiling!** |

Coming in at **₹2,224 per drone** gave us a comfortable ₹776 buffer per node for test jigs, spare propellers, and replacement motors when bench testing inevitably claims a few casualties.

---

## The Logic-Level MOSFET Discovery

Our biggest technical win of the week came from digging deep into the **AO3400A** MOSFET datasheet. 

In many DIY quadcopter projects, people try to drive standard power MOSFETs directly from 3.3V microcontroller pins, only to discover that the FETs never fully turn on. Standard gates often need 4.5V to 10V to reach saturation. If your gate isn't fully open, the MOSFET acts like a resistor, dissipates massive heat, drops the battery voltage, and burns out.

```
Standard FET @ 3.3V Logic:   Partially Open --> Hot! High Rds(on) --> Wasted battery
AO3400A FET @ 3.3V Logic:    Fully Saturated --> Cool (<30 mΩ) --> 98%+ Efficiency!
```

The AO3400A has a gate threshold voltage ($V_{gs(\text{th})}$) between **0.65V and 1.45V**. At our ESP32-S3’s 3.3V GPIO level, it is slammed completely into full saturation with an on-resistance under $30\text{ m}\Omega$. 

That meant we didn't need a heavy, expensive dedicated gate driver IC or level shifters. We can drive the gates directly from the ESP32 pins through a simple 100Ω damping resistor. That decision alone saved us ₹200 and about 2 grams of board weight per drone.

---

## The Weight Budget Reality

We also put our foot down on battery capacity. It’s always tempting to order bigger batteries—who doesn't want 15 minutes of flight time? But with 720 coreless motors, the math is brutal:

- Each 720 motor on 55mm props produces about **35–40g of static thrust** at full throttle.
- Four motors give us **~150g total thrust**.
- For crisp attitude recovery and snappy hover control, you want a thrust-to-weight ratio of at least **1.8:1 to 2.0:1**.
- That caps our maximum all-up weight (AUW) at **35–38 grams**.

Every extra 50 mAh on a battery adds 2–3 grams of dead weight. We finalized our battery order at **380 mAh (10.5g)**, which hits the exact sweet spot: ~4.5 minutes of hover time while keeping total flight weight right around 32 grams.

---

## The Orders Are In

By Friday afternoon, we had confirmed inventory and placed purchase orders across suppliers for all core electronics, spare motors, and battery packs. 

Next week, we enter every hardware developer's favorite purgatory: refreshing tracking numbers and preparing the test bench for unboxing day.
