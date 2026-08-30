# HiveLib – Monte Carlo Localization for PROS + LemLib

HiveLib is a Monte Carlo Localization (MCL) library designed to integrate cleanly with **PROS** and **LemLib**. It provides a probabilistic pose estimate using odometry, IMU data, and optional distance sensors, and can continuously synchronize that estimate with LemLib’s chassis pose.

The library is designed to be easy to drop into an existing codebase with minimal setup.

Developed by team [**70258A: Hivemind**](https://www.robotevents.com/teams/V5RC/70258A)

## How it works

### Motion Model

Each update reads the vertical tracking wheel and the IMU. Wheel ticks convert to linear distance (```ds```) through the wheel circumference, and the IMU gives a wrapped heading delta (```dth```). Every particle is then advanced by ```ds``` along its own heading and rotated by ```dth```, with Gaussian noise added via a Box–Muller transform.

The noise is motion-scaled rather than constant. Translation noise is ```SIGMA_TRANS_BASE_IN``` (0.10") plus ```SIGMA_TRANS_PER_IN``` (0.06") per inch traveled; heading noise is ```SIGMA_THETA_BASE_RAD``` plus ```SIGMA_THETA_PER_RAD``` per radian turned. A stationary robot barely spreads its cloud, while a long fast movement spreads it a lot. This matches how wheel slip and IMU drift actually accumulate — proportional to motion, not to time.

Prediction runs every 10 ms. Particles pushed outside the field bounds are respawned near the cloud mean with a 6" spread rather than deleted, which keeps the particle count fixed and avoids reallocating.

### Measurement model

Each distance sensor is registered with an (x, y, yaw) offset from the robot's center of rotation. For a given particle, the sensor's pose is transformed into field coordinates and a ray is cast to the field boundary to compute the distance that sensor should read if the robot were actually at that particle's pose. The residual between expected and measured is scored with a Gaussian (```SIGMA_DIST_IN``` = 1.25") and multiplied into the particle's weight.

Because the VEX field is a known 144" × 144" rectangle, the raycast is a closed-form ray–rectangle intersection rather than a march through an occupancy grid. That makes it O(1) per sensor per particle with no map to store, which is what makes 250 particles at 10 ms feasible on a V5 brain. The tradeoff is that only the outer walls are modeled since field elements aren't in the map.

Two gates protect the update. A hardware validity gate discards readings under 40 mm or over 2000 mm, outside the sensor's reliable range. An innovation gate discards any reading disagreeing with the expected wall distance by more than 10", which rejects most readings that hit a robot or field element instead of a wall.

Measurement runs every second loop (~20 ms) rather than every loop, since it's far more expensive than prediction and the extra rate buys little.

### Resampling

Resampling is gated on motion. It only runs if the robot has moved at least 0.75" or turned at least 3° since the last update. Without that gate, a stationary robot resamples repeatedly on near-identical weights and the cloud collapses to a handful of duplicated particles after which the filter can no longer represent alternative hypotheses.

The resample function is a a cumulative-sum with one random offset stepping through by 1/N: low-variance (systematic) resampling with O(N) time complexity.

The published pose is the weighted mean of the cloud. Headings are averaged as sine and cosine components recombined with ```atan2``` rather than averaged directly, which avoids the wraparound error where 179° and −179° average to 0°.

Confidence is derived from spatial spread rather than raw weights, mapping the cloud's standard deviation between 2" (tight, confident) and 18" (dispersed, likely lost) onto a 0–1 scale. Weights alone can look healthy while the cloud is dispersed, so spread is the more honest signal.

### Why 250 particles?

250 particles was a sweet spot found between accuracy and computational efficiency. As the field is only 12' x 12', iterating through 250 particles every 10ms is feasible for the V5 Brain. For local tracking, 250 particles is more than enough to handle minor variations, but since this ray-casting analytically against a rectangle instead of a grid, realistically it could be scaled up to 500 to 1000 particles if more accuracy was desired. Regardless, however, an odometry tracking wheel will be used as a motion model so when no valid readings come back, the filter runs prediction-only, which is dead reckoning

### Where it fails:

**Field symmetry**. The map is a square with only outer walls, so from the center, wall distances are close to identical under 90° rotations. The filter can converge confidently on a wrong hypothesis. Heading from the IMU is what disambiguates, which makes the IMU a hard dependency.

**No global relocalization**. reset() spreads particles with a 2" standard deviation around the given pose, so the filter is a local tracker. A wrong starting pose, or getting shoved several feet by another robot, is unrecoverable:  there's no mechanism to redistribute particles across the field.

**Dynamic obstacles**. The raycast assumes an empty rectangle. A sensor pointed at another robot or a mobile goal returns a short reading no particle predicts. The 10" innovation gate rejects the worst of these, but an obstacle producing an error just under the gate will corrupt weights.

**Range limits near center**. Readings beyond 2000 mm (~78") are discarded, and the walls are 72" away from field center. Near the middle, few or no sensors return valid readings and the filter degrades to prediction-only.

#### Design Notes with the full derivation are available in our [notebook](https://docs.google.com/document/d/1KD7MVZ-eUtZIvZQqOryz4Lbe8a_c98SCdaYAP6uw1Hc/edit?tab=t.r3y6zx7b96h3)

---

## Requirements

- **PROS kernel 4.2.x**
- **LemLib** (must already be installed in your project)

At minimum:
- 1 rotation sensor (tracking wheel or encoder)
- 1 IMU

Optional:
- Any number of PROS distance sensors for improved localization

---

## Installation (PROS)

HiveLib is distributed as a ZIP that can be installed directly into a PROS project.

### Steps

1. Download the latest release of **HiveLib** from the GitHub Releases page.
2. Extract the contents of the ZIP into the **root folder** of your PROS project.
3. In your `main.cpp`, add:

```cpp
#include "hivelib/api.hpp"
```

That is all that is required to install the library.

**Important:** HiveLib assumes LemLib is already installed. HiveLib does not modify or override any LemLib files.

---

## What You Must Configure

HiveLib is intentionally flexible, so a few components must be altered to match your robot.

---

## 1. Sensors and Distance Sensor Mounts

**File:** `hivelib/mcl_instance.hpp`

This file defines:
- The rotation sensor  
- The IMU  
- The list of distance sensors and their offsets  
- The global `MonteCarloLocalizer` instance (`mcl`)  

### Distance Sensor Offsets

Offsets are defined relative to the robot’s center of rotation.

- **x offset:** inches forward from robot center  
- **y offset:** inches left from robot center  
- **yaw offset:** sensor facing direction in radians  

Yaw reference:
- `0` = forward  
- `+π/2` = left  
- `−π/2` = right  
- `π` = backward  

Example:

```cpp
inline DistSensorMount mounts[] = {
    { &distFront,  0.0,  0.0,  0.0 },
    { &distLeft,   0.0,  0.0,  M_PI / 2.0 },
    { &distRight,  0.0,  0.0, -M_PI / 2.0 }
};
```

If you do **not** want to use distance sensors, set the mount count to `0` in the `MonteCarloLocalizer` constructor. This will instead only use odometry and the IMU like normal.

You may also replace the locally defined `pros::Rotation` and `pros::IMU` with sensors already defined in your drive system.

---

## 2. Field and Filter Configuration

**File:** `hivelib/mcl.hpp`

This file contains all tunable MCL parameters, including:
- Field dimensions  
- Number of particles  
- Motion noise  
- Heading noise  
- Distance sensor noise  
- Update rates  

Example:

```cpp
namespace mclcfg {
    constexpr double FIELD_MIN_X = 0.0;
    constexpr double FIELD_MIN_Y = 0.0;
    constexpr double FIELD_MAX_X = 144.0;
    constexpr double FIELD_MAX_Y = 144.0;

    constexpr int    N_PARTICLES = 250;
    constexpr double SIGMA_TRANS_IN = 0.35;
    constexpr double SIGMA_THETA_RAD = 0.015;
}
```

These values should be tuned for your robot’s drivetrain, wheel slip, and sensor quality.

---

## 3. LemLib Pose Synchronization

**File:** `hivelib/mcl_sync.hpp`

HiveLib can continuously push the MCL pose estimate into LemLib’s chassis pose. This allows all LemLib motion commands (pure pursuit, `moveToPoint`, etc.) to operate using the MCL estimate instead of raw odometry.

To enable pose synchronization:

1. Include your drive system header (the file where your LemLib `chassis` is defined).
2. Uncomment the pose sync line:

```cpp
chassis.setPose(p.x, p.y, p.theta * 180.0 / M_PI);
```

3. Start the synchronization task during the autonomous period:

In your `main.cpp` file, specifically the `autonomous` function, add the following two functions

```cpp
void autonomous() {
	startMclPoseSync();

    // insert the rest of your normal autonomous code here

	stopMclPoseSync();
}
```

This task typically runs at **10–20 ms** and keeps LemLib and MCL aligned in real time.

---

## 4. Resetting Pose at the Start of Autonomous

**File:** `hivelib/pose_utils.hpp`

HiveLib provides a helper function to reset **both LemLib and MCL** to the same starting pose.

After setting up your chassis, uncomment:

```cpp
chassis.setPose(x, y, heading_deg);
```

Then call at the start of your autonomous routine, call:

```cpp
resetAutonPose(0, 0, 0);
```

This ensures that both systems start from an identical pose reference.

---

## Typical Usage

A common autonomous setup looks like this:

1. Start the MCL task   
2. Reset Pose
3. Start LemLib pose synchronization  
4. Run autonomous normally using LemLib motion commands  

To start the main `mcl` task, include the following in your `initialize` function in `main.cpp`

```cpp
mcl.start();
```

---

## Notes

- HiveLib does **not replace** LemLib odometry; it adds to it.
- HiveLib can be used with:
  - Vertical-only odometry
  - Horizontal + vertical odometry
  - Motor encoders
  - Any combination with an IMU
- Distance sensors are optional but significantly improve long-term accuracy, especially during long autonomous routines or skills runs.

---

## License

MIT License.  
Free to use for competition, learning, and modification.
