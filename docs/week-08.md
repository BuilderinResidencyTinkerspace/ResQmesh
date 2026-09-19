# Week 8: Unboxing, Hard Trade-Offs & First Gyro Life

Monday morning felt like Christmas: three courier boxes sat on the workshop bench. Inside were our Seeed Studio XIAO ESP32-S3 boards, a tray of MPU9250 9-axis IMUs, bags of 720 coreless motors, 55mm propellers, and reels of AO3400A MOSFETs.

We immediately checked the 1S LiPo cells with a digital multimeter: **3.82V per cell**, right on nominal storage voltage. The build was officially underway.

---

## The Brutal Reality Check: Killing Onboard LoRa

Before we heated up the soldering iron, we had to make a tough engineering decision that we had been dreading for two weeks.

Our original concept proposal had included long-range **LoRa telemetry** on every drone. We loved the idea: kilometer-range swarm command links operating on 868/915 MHz. But holding the physical hardware in our hands shattered that fantasy.

```
Expected:  Tiny ESP32-S3 with built-in LoRa module (<1g extra)
Reality:   No such micro board exists in stock. 
           External SX1262 LoRa Breakout = 6.8 grams + bulky antenna!
```

On a 500g cinematic drone, 7 grams is nothing. On a micro-quadcopter whose total bare airframe weighs 7.4 grams and whose motors can only lift 50g comfortably, **a 7-gram breakout module is an anchor.** It would have reduced our flight time to under 90 seconds and ruined vehicle agility.

We made the executive call: **cut onboard LoRa from the micro flight nodes.**

Instead, we pivoted 100% of our swarm communication architecture to native **ESP-NOW**:
- Built directly into the ESP32-S3 silicon (zero extra grams).
- Sub-5 millisecond packet latency (over 10x faster than LoRa).
- Native peer-to-peer mesh broadcasting between nodes without needing external routers.
- LoRa isn't dead—it's being shifted to dedicated ground relay stations where weight doesn't matter.

---

## Wiring the IMU and the 400 kHz I2C Mystery

With that weight off our shoulders, we wired our first MPU9250 IMU breakout to the Seeed Studio XIAO ESP32-S3:
- `SDA` -> `GPIO5 / D4`
- `SCL` -> `GPIO6 / D5`
- `VCC` -> `3.3V`
- `GND` -> `Common GND`

We flashed a quick I2C scanner script. Within two seconds, the serial monitor printed:

```
Scanning I2C bus at 400 kHz...
Found device at address 0x68 (MPU9250 / MPU6050)
WHO_AM_I register check: 0x71 -> SUCCESS!
```

Then came the bug. 

The sensor would stream live gyro and accelerometer values for 15 seconds, and then the I2C bus would abruptly freeze solid. Resetting the ESP32 would sometimes bring it back, but touching the wires would cause it to lock up again.

```
ESP32 Internal Pull-ups (~45kΩ):  Weak! Slow rise time on 400 kHz clock --> I2C Bus Freeze!
Added External 4.7kΩ Pull-ups:   Sharp, crisp square waves --> Rock-solid stability!
```

We hooked up our logic analyzer and looked at the `SCL` clock line. Because the ESP32’s internal software pull-up resistors are weak (~45kΩ), the signal edges at 400 kHz were rounded into soft ramps instead of sharp square waves. Parasitic wire capacitance was stretching the clock transitions until the IMU missed an ACK bit.

We soldered two **4.7kΩ external pull-up resistors** directly between the SDA/SCL lines and the 3.3V rail. The waveform snapped into razor-sharp square edges, and the bus ran for 4 hours straight without dropping a single frame.

---

## First Attitude Tracking in 3D

With the I2C bus stabilized, we flashed our complementary filter and gyro calibration routine:
1. **Stationary Bias Sweep:** On bootup, the drone sits flat on the desk for 1.0 second, collecting 500 consecutive gyro samples. It averages out the earth's rotation and thermal bias, locking sensor zero-drift to under **$0.08^\circ/\text{s}$**.
2. **Complementary Fusion:**
   
   $$\text{Angle} = 0.98 \times (\text{Angle} + \omega \cdot dt) + 0.02 \times \text{AccelAngle}$$

We held the tiny board in our fingers and tilted it forward, backward, left, and right. On our laptop screen, the serial plotter tracked the roll and pitch angles with zero lag, snapping back to dead-level $0.0^\circ$ every time we set it down on the table.

---

## Next Up: The Power Stage

Next week, we move to the high-power side of the board: soldering four AO3400A MOSFETs, 1N5819 flyback diodes, and spinning our 720 coreless motors for the very first time.
