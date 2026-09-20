# Week 5: Schematics, Star Grounds & KiCad Workflows

Before you connect four brushed DC motors capable of pulling 8 Amps of combined burst current to a delicate 3.3V microcontroller, you’d better have your electrical paths figured out down to the millimeter. 

This week, we booted up KiCad to design the complete electrical schematic for ResQmesh. The goal wasn’t to send a board out to a commercial PCB fab—custom fab turnaround takes weeks and adds tooling fees that would violate our sub-₹3,000 budget. Instead, we used KiCad to design a bulletproof schematic and map out an exact point-to-point perfboard wiring layout that we could hand-solder directly onto the drone frame.

---

## The Silent Killer: Motor Flyback and Ground Bounce

If you've ever hooked a DC motor directly to an Arduino and watched the board reset the second the motor spins down, you've met **Back-EMF (Electromotive Force)**.

An electric motor is an inductor. When you turn a MOSFET ON, current builds up in the motor windings. When you turn the MOSFET OFF, the magnetic field collapses instantly, generating a massive reverse voltage spike:

$$V = -L \frac{di}{dt}$$

On a 3.7V battery, that inductive kick can easily spike to **30V or higher** for a few nanoseconds. If that voltage reaches your microcontroller or the gate of your MOSFET, it punches through the gate oxide and destroys the silicon instantly.

```
       [ +3.7V LiPo Rail ]
                |
          +-----+-----+
          |           |
        [Motor]   [1N5819 Diode] <--- Clamps reverse flyback safely!
          |           |
          +-----+-----+
                |
              [Drain]
      GPIO ---> [Gate]  AO3400A N-Channel MOSFET
              [Source]
                |
          [Power GND] (Battery Negative)
```

To kill this problem before it killed our hardware, our KiCad schematic incorporated three critical safety barriers:

1. **1N5819 Schottky Flyback Diodes:** Placed antiparallel across every motor terminal. When the MOSFET shuts off, the reverse inductive kick is immediately shunted through the diode back to the positive rail, clamping the spike safely to $V_{bat} + 0.45\text{ V}$.
2. **100Ω Series Gate Resistors:** Dampens high-frequency $LC$ ringing between the ESP32 pin capacitance and the MOSFET gate trace.
3. **10kΩ Gate Pull-Down Resistors:** Microcontroller GPIO pins float in high-impedance mode for a few milliseconds during bootup or firmware flashing. Without pull-down resistors, the gates pick up stray capacitive charge and turn the motors on unpredictably while the drone is sitting on your desk. The 10kΩ resistors hold the gates firmly at 0V until the firmware actively takes control.
<img width="540" height="437" alt="WhatsApp Image 2026-09-19 at 11 45 43 PM" src="https://github.com/user-attachments/assets/7cc1ad72-43da-4e19-ae21-c1a3b9ce56be" />
<img width="1132" height="797" alt="WhatsApp Image 2026-09-19 at 11 44 38 PM" src="https://github.com/user-attachments/assets/b0c4b477-8644-457c-9248-3e64ff5e6c37" />


---

## The Star Grounding Architecture

The other major trap on a micro quadcopter is **ground bounce**. 

When four coreless motors pulse at 20 kHz, several amps of current rush through the ground wire. If your IMU sensor shares that same ground trace, the resistance of the wire creates momentary voltage fluctuations ($\Delta V = I \cdot R$). 

To an MPU9250 listening on a 400 kHz I2C bus, a 200mV ground bounce looks like invalid data or a bus collision. The I2C state machine hangs, the flight loop misses its timing, and the drone crashes.

```
WRONG (Daisy Chain):
[Battery -] ------> [Motor GND] ------> [ESP32 GND] ------> [IMU GND]  <-- NOISE!
                                            ^
                       Motor current distorts IMU ground!

CORRECT (Star Ground):
                    +----> [Motor GND Returns] (Thick power trace)
                    |
[Battery Negative] -+----> [ESP32 Digital Ground]
                    |
                    +----> [IMU Ground] (Quiet, isolated analog rail)
```

We routed our schematic using strict **Star Grounding**: all high-current motor source returns converge at one single physical point—the negative battery solder pad. The ESP32 logic ground and IMU sensor ground branch off independently from that quiet origin point.

---

## Battery Sensing & Bulk Decoupling

To keep our LiPo from dropping below the danger threshold (3.2V under load), we added a simple 2:1 resistive divider:
- Two precision **100kΩ / 100kΩ** resistors step down the maximum 4.2V battery voltage to 2.1V.
- This feeds directly into the XIAO ESP32-S3’s ADC pin (`D0 / GPIO1`), allowing our flight firmware to monitor battery health in real time without exceeding the 3.3V ADC limit.

Finally, we placed a **470µF low-ESR bulk electrolytic capacitor** directly across the main battery pads to absorb sudden current surges during punch-outs, accompanied by **100nF ceramic decoupling caps** soldered directly across each motor tab to choke high-frequency brush arcing noise.

---

## Ready for the Iron

With our schematic validated through KiCad’s Electrical Rules Check (ERC) and our point-to-point perfboard footprint planned out, the circuit blueprint is set in stone.

Next week: the final component packages clear delivery, and we begin hand-wiring this entire circuit with tweezers, flux, and silicone wire.
