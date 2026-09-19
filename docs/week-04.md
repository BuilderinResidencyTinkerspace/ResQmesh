# Week 4: Refreshing Courier Tracking & Workbench Prep

If you've ever built physical hardware, you know the feeling: you have the designs finalized, the CAD printed, the schematics in your head, and your entire progress comes screeching to a halt because a parcel of microchips is stuck at a regional logistics hub three states away.

That was Week 4. But instead of twiddling our thumbs, we turned our workbench into an assembly line so that the moment the courier rings the doorbell, we hit the ground running.

---

## The Supply Chain Limbo

Every morning this week started the same way: opening tracking portals, watching the progress bars, and seeing "In Transit - Delayed due to logistics sorting." 

The Seeed XIAO ESP32-S3 boards, our MPU9250 IMU breakouts, and the batch of 720 coreless motors were all caught in transit across two different couriers. 

```
Component Wishlist:  [ XIAO ESP32-S3, MPU9250, 720 Motors, AO3400A FETs ]
Current Location:    [ "Out for sorting" ... for 5 days straight ]
```

We briefly considered rushing out to buy generic substitute parts from local hobby stores at three times the price, but that would defeat our core goal of building a strictly budgeted ₹2,200 platform. We decided to hold our ground, wait for our verified BOM to arrive, and put the downtime to work.

---

## Getting the Bench Ready

Nothing kills hardware momentum like realizing you're missing heatshrink, have a dirty soldering tip, or don't have a clean power supply when your parts finally arrive. We spent the week systematically prepping our build environment:

1. **Soldering Station Tune-Up:** 
   Soldering SOT-23 MOSFETs (which are smaller than a grain of rice) onto perfboard with bare copper wire requires a razor-sharp conical tip, fresh no-clean tacky flux, and 0.3mm fine solder wire. We cleaned and tinned our irons, verified temperature calibration, and set up our desktop magnifying loupe.

2. **Motor Testing Rig:**
   We rigged up an acrylic test block with 7mm vertical motor holes and a digital scale. When the motors arrive, we won't just trust the manufacturer's thrust specs—we'll mount them to the load cell, run a sweep from 10% to 100% PWM, and measure exact gram-thrust versus battery voltage sag.

3. **Battery Charging & Storage Station:**
   Micro LiPo cells require care. We wired up a 1S multi-port balance charging board, calibrated a digital multimeter for checking cell internal resistances, and verified that our 5V USB step-down regulators deliver clean, ripple-free power to our bench programmer.

4. **Wire Harness Preparation:**
   We cut, stripped, and pre-tinned dozens of ultra-flexible 30 AWG silicone wire leads for the I2C bus and motor gate signals, color-coded for power (red), ground (black), signals (yellow), and I2C lines (green/blue).

---

## Reflections from the Quiet Workbench

Hardware projects have a rhythm. There are frantic weeks where motors are screaming and hot glue is flying, and there are quiet weeks where progress is measured in clean workspaces, checked datasheets, and mental preparation.

Tracking updates show that the parcels have finally cleared the main distribution center and are scheduled for doorstep delivery early next week. 

Next week, the packages arrive, the soldering irons get hot, and we begin turning this pile of silicon into a living, breathing flight controller.
