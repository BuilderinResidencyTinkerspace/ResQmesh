# Week 7: Flying Inside the Terminal: The Host Simulator

There is an old, painful rule in drone development: **if your flight controller has a bug, it will explain that bug to you by crashing into your forehead at 50,000 RPM.**

Spinning untested PID code on raw hardware with spinning props is a recipe for snapped plastic, fried motors, and bruised fingers. So this week, before touching physical components, we built a complete C++ desktop flight dynamics simulator (`simulate_flight.cpp`) and automated test runner (`run_all_tests.cpp`) to fly the drone inside our computer terminal.

---

## Simulating Physics at 500 Hz

Our flight controller runs at a deterministic **500 Hz**—a new sensor read, attitude calculation, and motor command every **2,000 microseconds** ($2.0\text{ ms}$).

To simulate this on a PC without needing an ESP32 attached, we built lightweight hardware abstraction hooks into our core flight headers (`imu.h`, `receiver.h`, `motors.h`). The math doesn’t care whether an angular rate reading comes from a physical MPU9250 silicon wafer over I2C or from a virtual rigid-body physics integrator running in a C++ `for` loop.

```
       [ Simulated Pilot Stick Input ]      [ Simulated Wind Gust / Disturbance ]
                       \                                  /
                        v                                v
                 +-----------------------------------------------+
                 |        Cascaded Dual-Loop PID Controller      |
                 |      (Outer Angle Loop -> Inner Rate Loop)    |
                 +-----------------------------------------------+
                                         |
                                         v
                 +-----------------------------------------------+
                 |       Quad-X Anti-Saturation Mixer Math       |
                 +-----------------------------------------------+
                                         |
                                         v
                 +-----------------------------------------------+
                 |    Simulated Quadcopter Rigid Body Dynamics   |
                 |     (Moments of Inertia: Ixx, Iyy, Drag, dt)  |
                 +-----------------------------------------------+
```

We wrote a simulated flight environment that models:
- Quad-X motor thrust geometry and torque reactions ($M_1..M_4$).
- Physical moments of inertia along Roll, Pitch, and Yaw axes.
- Aerodynamic drag and gravitational acceleration ($9.81\text{ m/s}^2$).
- Ground effect and sensor noise.

---

## 7 Stress-Test Scenarios

We ran our virtual drone through 7 rigorous flight scenarios to deliberately break our algorithms before they could break real carbon fiber:

### 1. Bootup & Disarm Lockout
We simulated the first 500 milliseconds of power-on. Confirmed that motor PWM values remain strictly at `0`, and the system refuses to accept pilot commands until a clean disarm state is confirmed.

### 2. The Accidental Flip Arming Trap
What happens if the pilot tries to arm the drone while it's upside down or in someone's hand? We set the simulated pitch to $+30^\circ$ and injected an arming command. The safety watchdog caught it instantly: **arming was rejected** because vehicle tilt exceeded our $25^\circ$ safety gate.

### 3. Steady-State Level Hover
At 40% throttle (400 PWM), the drone lifted smoothly off the virtual floor, auto-stabilized roll and pitch to $<0.2^\circ$, and held all four motor outputs within 2 PWM units of each other.

### 4. The 15° Wind Shear Punch
While hovering at $0^\circ$ pitch, we suddenly injected a violent $+15^\circ$ nose-up aerodynamic disturbance. Within 120 milliseconds, the cascaded PID controller clamped down: front motors ($M_1, M_2$) spooled up, rear motors ($M_3, M_4$) dialed back, and the vehicle returned to perfectly level flight with zero overshoot.

### 5. High-Bank Slalom Rolls
We commanded sharp $\pm20^\circ$ roll steps. The inner angular rate loop ($K_p = 1.8, K_d = 0.04$) provided snappy differential torque response without ringing or sluggishness.

### 6. The 95% Throttle Punch & The Saturation Discovery
This was our biggest simulation breakthrough. When a pilot punches collective throttle to 95% (970 PWM) and simultaneously commands a hard roll to the right, standard mixer math does this:

$$M_1 = \text{Throttle} + \text{Roll} = 970 + 150 = 1120$$

Because 10-bit PWM caps out at `1023`, $M_1$ and $M_4$ clip at maximum power. With both left motors pegged at 1023, the flight controller has no headroom left to create differential thrust. **The drone loses all roll control and flips uncontrollably!**

```
WITHOUT Dynamic Anti-Saturation:
Motor 1: [==================== CLIP! 1023 ]  --> Lost attitude authority!
Motor 2: [==================== CLIP! 1023 ]

WITH Dynamic Anti-Saturation:
Excess Demand: 1120 - 1023 = 97 PWM
Dynamically subtract 97 from Collective Throttle!
Motor 1: [==================== 1023 ]  --> Attitude authority preserved!
Motor 2: [============== 726 ]       --> Differential torque maintained!
```

To fix this, we implemented **Dynamic Priority Anti-Saturation**: when any motor output exceeds 1023, the mixer calculates the excess demand and subtracts it equally from the collective throttle. The drone dips slightly in altitude for a split second, but **maintains 100% attitude stabilization.**

### 7. The Radio Dropout Failsafe
At $t = 3.5\text{s}$, we cut the simulated radio packet feed. After exactly 200 ms with no heartbeat, the watchdog timer tripped: the state machine transitioned instantly to `FAILSAFE`, cutting all four motors to `0` PWM to prevent a runaway flyaway.

---

## 41 Unit Tests Passed

We wrapped our algorithms in a standalone C++ unit test suite covering:
- Cascaded PID anti-windup clamping limits.
- Complementary attitude filter fusion accuracy ($\alpha = 0.98$).
- CRC16 packet checksum generation and corruption rejection.
- ADC battery moving-average digital filtering.

All **41 assertions passed with zero memory leaks** and an average loop execution time of under $450\ \mu\text{s}$—leaving massive headroom inside our $2000\ \mu\text{s}$ flight window.

Next week: the physical components are finally here. We unbox the silicon, verify the MPU9250 sensor over real I2C, and see how our theoretical math translates to physical reality.
