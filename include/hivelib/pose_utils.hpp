#pragma once
// #include "drive.hpp" // **Include the correct file for your drive system to get the distance sensor definitions
#include "hivelib/mcl_instance.hpp"
#include <cmath>

// Resets BOTH LemLib and MCL to the same starting pose
inline void resetAutonPose(double x, double y, double heading_deg) {
    chassis.setPose(x, y, heading_deg);
    mcl.reset(x, y, heading_deg * M_PI / 180.0);
    pros::delay(20);
}
