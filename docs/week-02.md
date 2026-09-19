# Week 2: Iterations, Snapped Arms, and Slicer Shrinkage

Before you write a single line of flight control code, your drone has to obey Newton. If the frame flexes when the motors spool up, your gyroscope will measure the bend of the plastic instead of the angle of the aircraft. When that happens, your PID loop tries to correct a phantom tilt, and the drone oscillates itself straight into the floor.

This week was all about fighting that mechanical reality on our 3D printer bed.

---

## Three Iterations and a Pile of Scrap PLA

We went into CAD with three competing design philosophies, sliced them, printed them, and put them through the wringer:

```
[ Iteration 1: Skeletal X ]   -->  Weight: ~5.0g  -->  Arm flex like a wet noodle
[ Iteration 2: Ducted Guard ]  -->  Weight: ~12.0g -->  A flying brick, choked airflow
[ Iteration 3: Unibody Micro ] -->  Weight: ~7.5g  -->  Rigid, clean CG, the sweet spot
```

### Iteration 1: The Skeletal Toothpick (~5.0g)
We got greedy on weight. We designed ultra-thin carbon-style arms with minimal trussing, weighing just over 5 grams. It looked great in CAD, but the moment we press-fit a test motor and spun it up, the arm vibrated like a tuning fork. Under full throttle, the motor pod deflected visibly by nearly 3 degrees. If we mounted our MPU9250 IMU on that, the vibration noise floor would completely swamp our attitude filter. Scrapped.

### Iteration 2: The Ducted Prop Guard (~12.0g)
Swinging to the opposite extreme, we designed fully enclosed circular ducts around each 55mm prop to make the drone indestructible for indoor testing. It was definitely tough—you could throw it against a wall—but at 12 grams, it ate over a third of our total vehicle weight budget before we even added a battery or flight controller. Worse, our bench tests showed that at micro scales, the duct walls created turbulent vortex recirculation around the prop tips, killing hover efficiency. Scrapped.

### Iteration 3: The Reinforced Unibody Micro-X (~7.5g)
This was the breakthrough. We abandoned round ducts and went for an open Quad-X geometry with an I-beam arm cross-section:
- **100% Solid Perimeters:** We set the slicer to print the motor arms with solid walls, eliminating hollow infill resonance.

- **Flight Controller Standoffs:** Built raised vibration-dampening mounting pads in the center to isolate the XIAO ESP32-S3 and IMU from motor shock.

---

## The 0.15mm Slicer Headache

The hardest lesson of the week had nothing to do with aerodynamics and everything to do with plastic shrinkage. 

Our coreless motors have an exact outer diameter of 7.00mm. On our first print of Iteration 3, the motor bores came off the bed measuring 6.85mm. We tried to press a motor in with our thumbs, heard a sickening *snap*, and split the entire motor pod down the layer lines.

```
Motor Outer Diameter:  7.00 mm
Raw 3D Print Bore:     6.85 mm  --> [CRACK!]
Tweaked with +0.12mm Hole Expansion: 7.02 mm  --> Snug friction fit!
```

Instead of re-modelling every motor pod in Fusion 360, we dug into slicer settings and calibrated **Horizontal Hole Expansion** by `+0.12mm`. On the next print, the bore came out at a perfect 7.02mm. The motors slid in with firm thumb pressure—snug enough that they wouldn't twist out under torque, but without stressing the layer adhesion.

---

## Where We Stand

By Sunday evening, we had a completed, rigid unibody frame sitting on the scale at exactly **7.4 grams**. Motor pods held firm, the battery snapped cleanly into the belly, and torsional stiffness passed our twist tests with flying colors.

Next week, we turn our attention to the electronics hunt: tracking down every resistor, diode, and MOSFET to build our flight hardware without breaching the ₹3,000 ceiling.
