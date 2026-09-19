# ESP32-DRONE: Production-Grade Custom Quadcopter Flight Controller

A modular, hard-real-time custom flight-controller firmware written from scratch in C++ for the **Seeed Studio XIAO ESP32-S3** and **MPU9250 9-axis IMU**, driving 4 × 720 brushed coreless DC motors through **AO3400A** N-channel MOSFET low-side switches, powered by a 1S 3.7V LiPo battery.

---

## 1. System Architecture Overview

```
                      +-----------------------------------+
                      |      ESP-NOW 2.4 GHz Receiver    |
                      +-----------------+-----------------+
                                        |
                                        v
+------------------+          +-------------------+          +------------------+
|   MPU9250 IMU    | -------> |  Attitude Filter  | -------> |  Outer Angle PID |
|  (I2C @ 400kHz)  |          | (Complementary)   |          |  (Roll & Pitch)  |
+------------------+          +-------------------+          +--------+---------+
         |                              |                             | Desired
         | Accel                        | Estimated                   | Angular
         | & Gyro                       | Roll, Pitch                 | Rates
         v                              v                             v
+------------------+          +-------------------------------------------------+
|  Zero-Bias Calib |          |             Inner Angular Rate PID              |
|  (500 Samples)   |          |             (Roll, Pitch, & Yaw)                |
+------------------+          +-----------------------+-------------------------+
                                                      |
                                                      v
+------------------+          +-------------------------------------------------+
|  Safety & State  | -------> |                Quad-X Motor Mixer               |
|  Machine Failsafe|          |      (Dynamic Attitude Anti-Saturation)         |
+------------------+          +-----------------------+-------------------------+
                                                      |
                                                      v
+------------------+          +-------------------------------------------------+
|  1S LiPo Monitor |          |             LEDC Hardware PWM Driver            |
| (12-bit ADC Avg) |          |               (20 kHz, 10-Bit Duty)             |
+------------------+          +-----------------------+-------------------------+
                                                      |
                                                      v
                                        4x AO3400A Low-Side MOSFETs
                                        4x 720 Brushed Coreless Motors
```

### Deterministic Dual-Core Execution
* **Core 1 (Flight Loop Task, Priority 24)**: Runs the strict **500 Hz deterministic control loop** (2000 µs period). Executes IMU burst read, attitude estimation, cascaded PID, mixer calculations, and LEDC updates in under 450 µs, leaving 1550 µs of deterministic headroom.
* **Core 0 (Comms & Background Task, Priority 5)**: Handles asynchronous non-blocking ESP-NOW packet reception, ADC battery filtering, visual status LED blinking, and the interactive Serial Debugging CLI.

---

## 2. Hardware Pinout (Seeed Studio XIAO ESP32-S3)

| Function | XIAO Pin | ESP32-S3 GPIO | Description / Notes |
| :--- | :--- | :--- | :--- |
| **I2C SDA** | D4 | `GPIO5` | 400 kHz Fast-Mode I2C data (external 4.7kΩ pull-up recommended) |
| **I2C SCL** | D5 | `GPIO6` | 400 kHz Fast-Mode I2C clock (external 4.7kΩ pull-up recommended) |
| **Motor 1 (FL)** | D2 | `GPIO3` | Front-Left motor (CW rotation) - LEDC PWM 20 kHz |
| **Motor 2 (FR)** | D3 | `GPIO4` | Front-Right motor (CCW rotation) - LEDC PWM 20 kHz |
| **Motor 3 (RR)** | D8 | `GPIO7` | Rear-Right motor (CW rotation) - LEDC PWM 20 kHz |
| **Motor 4 (RL)** | D9 | `GPIO8` | Rear-Left motor (CCW rotation) - LEDC PWM 20 kHz |
| **Battery ADC** | D0 | `GPIO1` | 1S LiPo voltage divider input (ADC1_CH0) |
| **Status LED** | Onboard | `GPIO21` | Active LOW diagnostic indicator |

---

## 3. Electrical & Driver Circuit Analysis

```
                        +1S LiPo Vbat (3.0V - 4.2V)
                                |
                   +------------+------------+
                   |                         |
               +---+---+                 +---+---+
               | 470uF | Bulk            | 10uF  | MCU
               | Cap   |                 | Cap   | Decoupling
               +---+---+                 +---+---+
                   |                         |
                  GND                       GND
                   |
     +-------------+-------------+
     |                           |
+----+----+                 +----+----+
| Motor + |                 | Cathode | 1N5819
| (720)   |                 |   [|]   | Schottky Diode
|         | <== [100nF] ==> |  /   \  | (in parallel with motor)
| Motor - |                 |  Anode  |
+----+----+                 +----+----+
     |                           |
     +-------------+-------------+
                   |
                   | Drain
              +----+----+
  ESP32-S3    |         | AO3400A N-Channel MOSFET
  GPIO ---[100R]-- Gate | (Vds=30V, Id=5.7A, Rds<30mOhm)
              |         |
              +--[10k]--+ Source
                   |         |
                  GND       GND (Power Star Ground)
```

### Critical Circuit Details:
1. **AO3400A Logic-Level Gate Drive**:
   - The AO3400A is an N-channel trench MOSFET with a threshold voltage $V_{gs(\text{th})}$ between 0.65V and 1.45V. Driven by the ESP32-S3 3.3V GPIO, its on-resistance $R_{ds(\text{on})}$ is less than $30\text{ m}\Omega$. At nominal motor current ($1.5\text{ A}$), conduction power loss is merely $P = I^2 R = (1.5)^2 \cdot 0.030 = 0.0675\text{ W}$, remaining cool without heatsinking.
2. **Gate Resistor ($100\ \Omega$ to $330\ \Omega$)**:
   - Placed in series between the ESP32-S3 GPIO and the AO3400A gate. The MOSFET input capacitance ($C_{iss} \approx 650\text{ pF}$) creates an instantaneous current surge when switching. The series resistor dampens $LC$ gate ringing and limits peak current into the microcontroller pin to $< 30\text{ mA}$.
3. **Gate Pull-Down Resistor ($10\text{ k}\Omega$ to $47\text{ k}\Omega$)**:
   - Placed directly between the MOSFET gate and GND. **Crucial Safety Requirement**: During ESP32-S3 boot, flashing, or reset, GPIO pins enter high-impedance (floating) states. Without this pull-down, gate leakage current would charge the gate capacitance, causing the motors to spin uncontrolled during power-up!
4. **Low-Side Switching Topology**:
   - **Source**: Connected directly to the low-impedance battery ground.
   - **Drain**: Connected to the Motor negative terminal ($-$).
   - **Motor ($+$)**: Connected directly to the battery positive rail ($V_{bat}$).
5. **1N5819 Schottky Flyback Diodes**:
   - Placed antiparallel across the motor terminals: **Cathode** to $V_{bat}$ (Motor $+$), **Anode** to MOSFET Drain (Motor $-$).
   - Coreless brushed motors are inductive loads. When the MOSFET turns OFF, the magnetic field collapses, generating a high-voltage inductive kick:
     $$V_{\text{spike}} = -L \frac{di}{dt}$$
   - Without the diode, this flyback spike exceeds the 30V breakdown voltage of the AO3400A, destroying the FET. The 1N5819 Schottky diode has an ultra-fast reverse recovery time and low forward drop ($V_f \approx 0.45\text{ V}$), clamping the drain voltage safely to $V_{bat} + 0.45\text{ V}$.
6. **$470\ \mu\text{F}$ Bulk Capacitor Placement**:
   - Placed across the main battery power rail as close as possible to the motor FET source pads. When 4 coreless motors punch out at full throttle, transient current steps can reach $8\text{ A}$, causing severe battery voltage sag. The bulk capacitor provides local charge storage, preventing ESP32-S3 brownout resets.
7. **$100\text{ nF}$ Ceramic Decoupling Capacitors**:
   - Solder a $100\text{ nF}$ ceramic capacitor directly across each motor's terminal solder tabs. Mechanical brush commutation causes continuous high-frequency RF arcing. The ceramic capacitor shunts this RF noise at the source, preventing electromagnetic interference from disrupting the MPU9250 I2C bus.
8. **Star Grounding**:
   - High-current motor return paths must be routed directly back to the battery negative pad. Never route high-current motor ground return through the logic ground traces of the MPU9250 or ESP32-S3!

---

## 4. Motor Numbering & Quad-X Mixer Layout

```
                  FRONT (Nose)
                       ^
                       |
         M1 (FL, CW)   |   M2 (FR, CCW)
            \          |          /
             \         |         /
              \        |        /
      <--------+-------X-------+--------> (Roll Right: +)
              /        |        \
             /         |         \
            /          |          \
        M4 (RL, CCW)   |   M3 (RR, CW)
                       |
                  REAR (Tail)
```

### Motor Direction & Torque Derivation
* **Motor 1 (Front-Left)**: Spins **Clockwise (CW)**. Produces **Counter-Clockwise (CCW)** reactive torque on the airframe.
* **Motor 2 (Front-Right)**: Spins **Counter-Clockwise (CCW)**. Produces **Clockwise (CW)** reactive torque on the airframe.
* **Motor 3 (Rear-Right)**: Spins **Clockwise (CW)**. Produces **Counter-Clockwise (CCW)** reactive torque on the airframe.
* **Motor 4 (Rear-Left)**: Spins **Counter-Clockwise (CCW)**. Produces **Clockwise (CW)** reactive torque on the airframe.

### Matrix Equations:
$$M_1 (\text{FL}) = \text{Throttle} + \text{Roll} - \text{Pitch} - \text{Yaw}$$
$$M_2 (\text{FR}) = \text{Throttle} - \text{Roll} - \text{Pitch} + \text{Yaw}$$
$$M_3 (\text{RR}) = \text{Throttle} - \text{Roll} + \text{Pitch} - \text{Yaw}$$
$$M_4 (\text{RL}) = \text{Throttle} + \text{Roll} + \text{Pitch} + \text{Yaw}$$

### Dynamic Priority Anti-Saturation
When high throttle coincides with large roll/pitch/yaw corrections, calculated motor outputs can exceed 1023 (100% duty). If outputs were simply clipped, differential torque would be lost, causing uncontrolled tumbling.
The firmware calculates the excess:
$$\text{Excess} = \max(M_1, M_2, M_3, M_4) - 1023$$
If $\text{Excess} > 0$, it subtracts this value equally from all four motors, maintaining full roll/pitch/yaw attitude control authority at the expense of slight climb rate.

---

## 5. Control Loop Architecture & Mathematics

```
Pilot Stick Input
       |
       v
+------------------+
| Outer Angle Loop |  Error = Target_Angle - Estimated_Angle
|     (P Loop)     |  Target_Rate = Kp_Angle * Error
+--------+---------+
         | Target_Rate
         v
+------------------+
| Inner Rate Loop  |  Error = Target_Rate - Measured_Gyro_Rate
|    (PID Loop)    |  Output = Kp * e + Ki * ∫e dt + Kd * d(e_filtered)/dt
+--------+---------+
         | Axis Torque Demand
         v
+------------------+
| Quad-X Mixer     |  Mixes Throttle + Roll + Pitch + Yaw
| Anti-Saturation  |  Preserves attitude authority during saturation
+--------+---------+
         | Motor 1..4 Duty (0-1023)
         v
+------------------+
| 20 kHz LEDC PWM  |  Ultrasonic frequency drives AO3400A MOSFETs
+------------------+
```

### Complementary Filter Formula
$$\theta_{k} = \alpha \cdot (\theta_{k-1} + \omega_{\text{gyro}} \cdot \Delta t) + (1 - \alpha) \cdot \theta_{\text{acc}}$$
* Default $\alpha = 0.98$ (configured in [config.h](file:///e:/ResQmesh/code/ESP32-DRONE/main/config.h)).
* Accelerometer roll: $\phi_{\text{acc}} = \text{atan2}(a_y, a_z) \cdot \frac{180}{\pi}$
* Accelerometer pitch: $\theta_{\text{acc}} = \text{atan2}(-a_x, \sqrt{a_y^2 + a_z^2}) \cdot \frac{180}{\pi}$
* Dynamic acceleration rejection: If $||\vec{a}||$ deviates outside $[0.75g, 1.25g]$, the filter temporarily ignores accelerometer corrections to avoid centripetal flight disturbances.

---

## 6. Battery Monitor & Voltage Divider

Do **NOT** connect 1S LiPo voltage directly to an ESP32-S3 ADC pin (maximum input voltage is 3.1V with 12dB attenuation).

### Resistive Divider Circuit
```
   1S LiPo Vbat (3.0V - 4.2V)
               |
             [100k] R1 (1% Metal Film)
               |
               +-----> XIAO D0 (GPIO1 / ADC1_CH0)  (Max 2.10V)
               |
             [100k] R2 (1% Metal Film)
               |
              GND
```
* **Divider Ratio**: $k = \frac{R_1 + R_2}{R_2} = \frac{100\text{k} + 100\text{k}}{100\text{k}} = 2.00$
* At full battery ($4.20\text{ V}$), the pin sees $2.10\text{ V}$, well within the linear conversion range of the ESP32-S3 ADC.
* Moving average filter over 16 samples dampens voltage ripple from motor PWM switching.

---

## 7. Interactive Serial Debugging CLI

Connect to the USB-C serial port at **115200 baud**. Type any command:

| Command | Description |
| :--- | :--- |
| `status` | System state, IMU status, receiver link, battery voltage/%, loop frequency and execution timings |
| `imu` | Real-time 3-axis accelerometer, gyroscope, and die temperature readings |
| `attitude` | Current estimated roll, pitch, roll rate, pitch rate, and yaw rate |
| `pid` | Displays active cascaded PID gains, error tracking, and integral levels |
| `battery` | Battery voltage, capacity percentage, and warning/critical flags |
| `motors` | Current PWM duty cycles (0–1023) sent to M1, M2, M3, M4 |
| `calibrate` | Executes 500-sample zero-bias calibration (requires drone disarmed and motionless) |
| `disarm` | Immediately stops all motors and resets PID integrators |
| `test_motor <1-4> <0-25>`| Bench tests a single motor at safe duty (requires `#define MOTOR_TEST_ENABLED 1`) |
| `sim <roll> <pitch>` | Injects synthetic orientation angles into the pipeline to test PID/mixer response |
| `help` | Lists all console commands |

---

## 8. Build, Flash, and Testing Instructions

### 1-Click Automated Windows Flasher (Arduino CLI)
The project includes a flasher script that auto-detects your connected XIAO ESP32-S3 port and prompts for confirmation:
```cmd
# Auto-detects port or prompts to select:
.\code\ESP32-DRONE\flash_drone.bat

# Or explicitly specify your COM port:
.\code\ESP32-DRONE\flash_drone.bat COM14
```

### ESP-IDF CLI Build
```bash
cd code/ESP32-DRONE
idf.py set-target esp32s3
idf.py build
idf.py -p COM_PORT flash monitor
```

### PlatformIO Build
```bash
cd code/ESP32-DRONE
pio run -t upload -t monitor
```

### Running Automated Host Unit Tests (No hardware required)
```bash
cd code/ESP32-DRONE/tests
.\run_tests.bat
```
*(Executes all 41 test assertions verifying PID, complementary filter, mixer anti-saturation, CRC16 packet validation, battery calculations, and failsafe state machine).*

---

## 9. PID Tuning Guide

1. **Bench Setup**: Remove propellers. Secure drone in a test gimbal or pivot.
2. **Inner Rate Loop ($K_p$, $K_d$, $K_i$) First**:
   - Switch transmitter to **Rate Mode** (`mode = 1`) or set outer loop $K_p = 0$.
   - Slowly increase Rate $K_p$ (e.g. from 0.3 to 0.7) until the drone responds crisply to manual rate demands.
   - If motors begin to buzz or vibrate at high frequency, increase Rate $K_d$ slightly (e.g. 0.02) to dampen overshoot, or increase the D-filter time constant $\tau$.
   - Add Rate $K_i$ (e.g. 0.3 to 0.5) to eliminate steady-state drift.
3. **Outer Angle Loop ($K_p$) Second**:
   - Switch transmitter to **Angle Mode** (`mode = 0`).
   - Increase Angle $K_p$ (e.g. 3.0 to 5.0) until the drone levels itself rapidly when sticks are released. If the drone oscillates slowly (1–2 Hz wobble), lower Angle $K_p$.

---

## 10. Safe First-Flight Commissioning Checklist

> [!CAUTION]
> **NEVER ATTACH PROPELLERS TO A NEW FLIGHT CONTROLLER UNTIL STEPS 1 THROUGH 13 HAVE ALL BEEN VERIFIED.**

- [ ] **Step 1: REMOVE PROPELLERS**. Do not proceed with propellers attached under any circumstances.
- [ ] **Step 2: Verify Motor Wiring**. Confirm AO3400A Source is to GND, Drain is to Motor ($-$), Motor ($+$) is to $V_{bat}$, and 1N5819 diode cathode is to $V_{bat}$.
- [ ] **Step 3: Verify Motor Numbering**. Confirm M1 is Front-Left, M2 is Front-Right, M3 is Rear-Right, M4 is Rear-Left.
- [ ] **Step 4: Verify Motor Rotation Direction**.
  - Enable `#define MOTOR_TEST_ENABLED 1` in `config.h`.
  - In serial console, pulse each motor:
    - `test_motor 1 5` -> M1 must spin **Clockwise (CW)**.
    - `test_motor 2 5` -> M2 must spin **Counter-Clockwise (CCW)**.
    - `test_motor 3 5` -> M3 must spin **Clockwise (CW)**.
    - `test_motor 4 5` -> M4 must spin **Counter-Clockwise (CCW)**.
- [ ] **Step 5: Verify IMU Axes**. Run `imu` in console. Tilting drone nose down must show negative $a_x$. Tilting right wing down must show positive $a_y$. Inverting drone must show negative $a_z$.
- [ ] **Step 6: Verify Calibration**. Place drone flat and motionless. Run `calibrate`. Confirm offsets are computed and gyro bias is $< 0.1\text{ deg/s}$.
- [ ] **Step 7: Verify Attitude Signs**. Run `attitude` in console.
  - Tilt drone 20° right wing down -> Roll must report $\approx +20^{\circ}$.
  - Tilt drone 20° nose up -> Pitch must report $\approx +20^{\circ}$.
- [ ] **Step 8: Verify Roll Mixer Reaction**.
  - Arm drone at low throttle (bench test without props).
  - Manually tilt drone right wing down.
  - Right motors (M2, M3) must automatically speed up and left motors (M1, M4) must slow down to resist the tilt.
- [ ] **Step 9: Verify Pitch Mixer Reaction**.
  - Manually pitch drone nose down.
  - Front motors (M1, M2) must automatically speed up and rear motors (M3, M4) must slow down to restore level attitude.
- [ ] **Step 10: Verify Yaw Mixer Reaction**.
  - Manually twist drone clockwise.
  - CW motors (M1, M3) must speed up to impart CCW counter-torque.
- [ ] **Step 11: Verify Failsafe**.
  - With drone armed at low throttle, power off the transmitter.
  - All 4 motors must **INSTANTLY STOP** within 200 ms.
- [ ] **Step 12: Verify Pre-Arm Safety Gate**.
  - Raise transmitter throttle above 10%. Try arming. Drone must reject arming.
  - Tilt drone $>25^{\circ}$. Try arming. Drone must reject arming.
- [ ] **Step 13: Tethered Low-Throttle Test**.
  - Secure drone loosely on a tether with props attached. Test throttle response at $\le 15\%$ throttle.
- [ ] **Step 14: Controlled Flight**.
  - Only after Steps 1–13 are 100% verified, place drone on an open, soft surface (grass/carpet) and perform initial low-altitude hover test.
