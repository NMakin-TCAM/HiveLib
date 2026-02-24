#pragma once
#include "hivelib/mcl.hpp"
#include "api.h" // remove this if you replace the rotation and IMU sensors with pre defined variables from your drive system

// #include "drive.hpp" // **Include the correct file for your drive system to get the distance sensor definitions

// Distance Sensor offsets
inline DistSensorMount mounts[] = {
    // { &distFront, +x_off, +y_off, yaw_off_rad }
    // yaw: 0 forward, +pi/2 left, -pi/2 right
    /*
    x_off = inches forward from robot center of rotation
    y_off = inches left of robot center of rotation
    yaw_off = sensor facing direction relative to robot looking forward (0 means forward, +pi/2 means left, -pi/2 means right, pi = backward)
    
    Example:
    { &distFront, 0.0, 0.0, 0.0 },          // front sensor
    { &distLeft, 0.0, 0.0, M_PI / 2.0 },   // left sensor
    { &distRight, 0.0, 0.0, -M_PI / 2.0 } // right sensor
    */
};


inline pros::Rotation rot(1);
inline pros::IMU imu(2);

inline MonteCarloLocalizer mcl(
    &rot, // you can replace this with a pre defined rotation sensor and delete the one defined above
    &imu, // you can replace this with a pre defined IMU and delete the ones defined above
    mounts,
    0,      // Number of distance sensors (0 means it'll just use IMU and odom)
    2.0     // tracking wheel diameter in inches
);
