#pragma once
// #include "drive.hpp" // **Include the correct file for your drive system to get the distance sensor definitions
#include "hivelib/mcl_instance.hpp"
#include <cmath>

// Resets BOTH LemLib and MCL to the same starting pose
inline void resetAutonPose(double x, double y, double heading_deg) {
    // chassis.setPose(x, y, heading_deg); // Replace chassis with your LemLib drive object and then uncomment
    mcl.reset(x, y, heading_deg * M_PI / 180.0); // Uncomment once you have an MCL instance named mcl in mcl_instance.hpp
    pros::delay(20);
}
