# Week 9: SOT-23 Tweezers, Solder Fumes & 20 kHz Ultrasonic Silence

If you want to test your soldering sanity, try hand-soldering four surface-mount SOT-23 MOSFETs—each about the size of a sesame seed—onto a perfboard scrap using tweezers and a magnifying glass while breathing through a fume extractor.

This week was the trial by fire: building the high-current motor driver stage, fighting acoustic motor whine, and spinning all four 720 coreless motors for the very first time.

---

## Surgery with SOT-23s

To keep our flight controller as light as possible, we opted against bulky breakout boards. We hand-wired the **AO3400A MOSFETs** directly on a 20mm × 20mm perfboard section:

```
        +-----------------------------------------------+
        |  [M1 FET]   [M2 FET]   [M3 FET]   [M4 FET]   |
        |   (FL CW)   (FR CCW)   (RR CW)    (RL CCW)    |
        |                                               |
        |  1N5819 Schottky Clamping Diodes Across Each  |
        |  100Ω Gate Series + 10kΩ Gate Pull-Downs      |
        |  470µF Low-ESR Bulk Rail Capacitor            |
        +-----------------------------------------------+
```

<img width="300" height="400" alt="20260915_001343" src="https://github.com/user-attachments/assets/efbc844c-e978-4f89-b35a-9c03be6d6b7c" />


The process required steady hands and a lot of tacky flux:
1. We bent the Source pins of all four FETs downward and soldered them to a thick, solid-copper bare ground rail running along the perimeter (our **Star Ground**).
2. We soldered tiny **100Ω resistors** directly to the floating Gate pins, connecting them via 32 AWG flexible enamel wires back to the XIAO ESP32-S3's PWM pins (`D1`, `D2`, `D3`, `D6`).
3. We tacked **1N5819 Schottky diodes** across the motor output pads, double-checking cathode band orientations to avoid shorting the battery directly to ground.
4. We bridged a **470µF low-ESR electrolytic capacitor** directly across the main LiPo input pads to swallow transient voltage dips during aggressive throttle punches.

Before plugging in a battery, we spent thirty minutes with our multimeter on continuity mode, probing every single adjacent trace. **Zero shorts.**

---

## The Ear-Ringing 1 kHz Mosquito Whine

We wrote a minimal PWM test script to spin Motor 1 at 25% duty cycle. We plugged in the 1S LiPo, sent the test command, and were immediately greeted by an ear-splitting, piercing squeal that sounded like an angry mosquito directly inside our eardrums.

At standard microcontroller PWM frequencies (1 kHz to 4 kHz), the rapid pulsing of current through the motor windings causes the motor casing and armature coils to physically vibrate at audio frequencies. It turns the motors into tiny mechanical loudspeakers.

```
Standard PWM (1 kHz - 4 kHz):    [EEEEEEEEEEEEEEEE!] --> Painful acoustic resonance!
Ultrasonic PWM (20 kHz):         [Dead Silence...]    --> Above human hearing range!
```

We went straight into the ESP-IDF **LEDC PWM peripheral configuration** and reconfigured our timer:
- **Frequency:** Bumped from 2 kHz straight to **20 kHz**.
- **Resolution:** Set to **10-bit** (`0` to `1023` duty steps).

We hit enter. 

The squeal vanished completely. In its place was eerie, dead silence—just the quiet *whoosh* of air as the 55mm propeller spun up smoothly on the test stand. 20 kHz is safely above the human hearing limit (~18–19 kHz for adults), giving us silky-smooth torque delivery without the deafening whine.

---

## Measuring Real Efficiency: 45 Millivolts

Once the motors were spinning quietly, we brought out the oscilloscope and multimeter to verify our MOSFET saturation math from Week 3.

At full 100% throttle, a 720 coreless motor draws approximately **1.5 Amps** of continuous current. We placed our probes across the Drain and Source pins of the AO3400A:

$$V_{ds} \approx 45\text{ mV} \quad (0.045\text{ V})$$

Using Ohm's law, we calculated the real-world in-circuit on-resistance:

$$R_{ds(\text{on})} = \frac{V_{ds}}{I} = \frac{0.045\text{ V}}{1.5\text{ A}} = 0.030\ \Omega \quad (30\text{ m}\Omega)$$

And the thermal power dissipation:

$$P_{\text{loss}} = I^2 \cdot R = (1.5\text{ A})^2 \times 0.030\ \Omega = \mathbf{0.067\text{ Watts}}$$

At less than **70 milliwatts** of heat loss, the MOSFETs stayed completely cold to the touch even after 3 minutes of continuous full-throttle bench testing. That means **$>98\%$ of our battery energy is going straight to the propellers**, not wasted as heat in the driver stage.

---

## Checking Quad-X Rotation Directions

Finally, we used our custom interactive USB serial CLI to spin each motor individually and verify rotation directions against our Quad-X mixer geometry:

```
    M1 (Front-Left, CW)       M2 (Front-Right, CCW)
            \                       /
             \                     /
              +-------------------+
              |     ResQmesh      |
              +-------------------+
             /                     \
            /                       \
    M4 (Rear-Left, CCW)       M3 (Rear-Right, CW)
```

- `test_motor 1 5` -> Front-Left spun clockwise (CW).
- `test_motor 2 5` -> Front-Right spun counter-clockwise (CCW).
- `test_motor 3 5` -> Rear-Right spun clockwise (CW).
- `test_motor 4 5` -> Rear-Left spun counter-clockwise (CCW).

Every single channel responded with zero jitter. The hardware is built, tested, and electrically verified.

Next week, we flash our complete dual-core FreeRTOS flight firmware, mount the electronics to the frame, and see if our drone can stabilize itself in the physical world.
