# Week 2

**Goal this week:** Design, 3D print, test, and finalize a lightweight, rigid micro-quadcopter airframe tailored for 720 coreless motors and the XIAO ESP32-S3 flight controller.

## What we did

- Designed multiple CAD iterations of micro-quadcopter frames to evaluate weight, structural rigidity, and motor retention.
- Fabricated and tested several 3D-printed physical prototypes:
  - **Iteration 1 (Minimalist Skeletal X)**: Extremely lightweight (~5g), but exhibited excessive arm flex under motor vibration.
  - **Iteration 2 (Duct/Prop-Guard Integrated Frame)**: High crash protection, but added excessive weight (~12g) and caused aerodynamic turbulence in hover.
  - **Iteration 3 (Reinforced Unibody Micro-X)**: Optimized arm cross-section, low weight (~7.5g), rigid motor pods, and an integrated central bay for the flight controller and battery.
- Conducted motor mount tolerance tests: calibrated the 7mm cylindrical motor sockets for a secure press-fit grip that prevents motor slippage during high-RPM punch-outs.
- Evaluated Center of Gravity (CG) alignment by placing dummy weights representing the 1S LiPo, Seeed XIAO board, and wiring.
- Selected and fixed **Iteration 3 (Reinforced Unibody Micro-X)** as the final airframe design for the drone fleet.

## Problems and blockers

- **Arm Resonance & Flex**: Initial lightweight prints suffered from flexural vibrations that could induce severe gyro noise into the MPU9250 IMU attitude filter.
- **3D Print Shrinkage & Tolerances**: The 7.0mm motor bores printed slightly undersized (6.85mm) on initial slicer settings, requiring slicer compensation (horizontal hole expansion) to prevent cracking when inserting the coreless motors.
- **Weight vs. Durability**: Balancing structural strength for hard landings while staying within the tight thrust budget of 720 motors and 55mm propellers.

## Decisions

- **Finalized Airframe**: Standardized on the **Reinforced Unibody Micro-X** frame design for all swarm nodes.
- **Print Settings & Material**: Selected high-infill PLA / PETG with 100% solid perimeters along the motor arms to minimize acoustic and mechanical resonance.
- **Battery Placement**: Fixed an under-belly snap-in battery cage to place the 1S LiPo directly beneath the thrust intersection plane, ensuring ideal neutral CG.
- **Modular Component Mounting**: Incorporated standard standoff locations to cleanly mount the XIAO ESP32-S3 and IMU sensor board isolated from motor vibrations.

## Next week


- Mount the electronics and motors onto the finalized frame.
- Perform initial bench tests: verify motor spin directions and PWM speed response.

## Links

- CAD Models & Designs: [cad/](file:///e:/ResQmesh/cad)
- Project Overview: [docs/index.md](index.md)
