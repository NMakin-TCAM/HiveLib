#pragma once
#include "pros/rtos.hpp"
// #include "drive.hpp" // **Include the correct file for your drive system to get the distance sensor definitions
#include "hivelib/mcl_instance.hpp"

// Keeps LemLib's chassis pose locked to the MCL estimate
inline pros::Task* mclSyncTask = nullptr;

inline void startMclPoseSync() {
  if (mclSyncTask) return;

  mclSyncTask = new pros::Task([] {
    while (true) {
      const auto p = mcl.getPose();

      // If your MCL pose theta is radians, convert to degrees here:
      chassis.setPose(p.x, p.y, p.theta * 180.0 / M_PI);

      pros::delay(10); // 10–20ms is fine
    }
  }, "MCL->Chassis Sync");
}

inline void stopMclPoseSync() {
  if (!mclSyncTask) return;
  delete mclSyncTask;
  mclSyncTask = nullptr;
}
