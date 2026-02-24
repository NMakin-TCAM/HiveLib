# HiveLib – Monte Carlo Localization for PROS + LemLib

HiveLib is a Monte Carlo Localization (MCL) library designed to integrate cleanly with **PROS** and **LemLib**. It provides a probabilistic pose estimate using odometry, IMU data, and optional distance sensors, and can continuously synchronize that estimate with LemLib’s chassis pose.

The library is designed to be easy to drop into an existing codebase with minimal setup.

Developed by team 70258A: Hivemind

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