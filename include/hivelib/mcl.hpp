#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include <algorithm>


// ======= USER-CONFIG =======
namespace mclcfg {
    constexpr double FIELD_MIN_X = 0.0;
    constexpr double FIELD_MIN_Y = 0.0;
    constexpr double FIELD_MAX_X = 144.0;
    constexpr double FIELD_MAX_Y = 144.0;

    // Particle filter tuning
    constexpr int    N_PARTICLES = 250;
    constexpr double INIT_X = 72.0;        // start pose guess (in)
    constexpr double INIT_Y = 72.0;
    constexpr double INIT_THETA = 0.0;     // radians

    // Noise (tune later)
    constexpr double SIGMA_TRANS_IN = 0.35;   // inches of motion noise per update
    constexpr double SIGMA_THETA_RAD = 0.015; // radians of heading noise per update
    constexpr double SIGMA_DIST_IN = 1.25;    // distance sensor measurement noise (in)

    // Reject distance readings outside this range (mm from PROS Distance)
    constexpr int DIST_MIN_MM = 40;
    constexpr int DIST_MAX_MM = 2000;

    // Update rates
    constexpr int UPDATE_MS = 10;            // PF loop rate (predict every loop)
    constexpr int MEAS_UPDATE_EVERY_N = 2;   // do distance update every N loops

    // Don't resample if the robot hasn't moved at least this much since the last resample (to save processing)
    constexpr double RESAMPLE_MIN_DS_IN = 0.75;        // inches
    constexpr double RESAMPLE_MIN_DTH_RAD = 3.0 * M_PI / 180.0; // ~3 degrees

    // Defining confidence thresholds based on spread
    constexpr double CONF_GOOD_STD_IN = 2.0;   // ~localized
    constexpr double CONF_BAD_STD_IN  = 18.0;  // ~lost

    constexpr double DIST_GATE_IN = 10.0; // accept if within ±10"

    constexpr double SIGMA_TRANS_BASE_IN = 0.10;
    constexpr double SIGMA_TRANS_PER_IN  = 0.06;  // extra noise per inch moved

    constexpr double SIGMA_THETA_BASE_RAD = 0.004;
    constexpr double SIGMA_THETA_PER_RAD  = 0.15; // extra noise per rad turned

    // Respawn spread for out-of-bounds particles
    constexpr double OOB_RESPAWN_XY_SIGMA_IN = 6.0;
    constexpr double OOB_RESPAWN_TH_SIGMA_RAD = 0.20;
}

// A single distance sensor mounting description
struct DistSensorMount {
    pros::Distance* sensor;  // pointer to PROS distance sensor
    double x_off;            // inches (robot frame): +x forward, +y left
    double y_off;            // inches
    double yaw_off;          // radians: 0 = forward, +CCW
};

class MonteCarloLocalizer {
public:
    explicit MonteCarloLocalizer(
        pros::Rotation* rot,
        pros::IMU* imu,
        const DistSensorMount* mounts,
        int mountCount,
        double trackingWheelDiamIn // 2.0 for 2" omni etc.
    );

    void start();                 // starts internal task
    void stop();

    void reset(double x, double y, double thetaRad);
    lemlib::Pose getPose() const; // x,y in inches; theta in radians (LemLib Pose uses radians)
    double confidence() const;    // 0..1-ish heuristic

private:
    struct Particle { double x, y, th, w; };

    void loop_();
    void predict_(double ds, double dth);
    void measurementUpdate_();
    void resample_();

    // helpers
    static double wrapAngle_(double a);
    static double gaussProb_(double err, double sigma);
    static bool   rayRectDist_(double ox, double oy, double dx, double dy,
                               double xmin, double ymin, double xmax, double ymax,
                               double& outDist);

    pros::Rotation* m_vertRot;
    pros::IMU* m_imu;
    const DistSensorMount* m_mounts;
    int m_mountCount;
    double m_trackWheelDiamIn;

    mutable pros::Mutex m_poseMutex;
    lemlib::Pose m_pose{mclcfg::INIT_X, mclcfg::INIT_Y, mclcfg::INIT_THETA};
    double m_conf = 0.0;


    // PF state
    Particle m_particles[mclcfg::N_PARTICLES];

    // odom bookkeeping
    double m_lastVertDeg = 0.0;
    double m_lastImuRad = 0.0;

    pros::Task* m_task = nullptr;
    bool m_running = false;

    double m_meanX = mclcfg::INIT_X;
    double m_meanY = mclcfg::INIT_Y;
    double m_meanTh = mclcfg::INIT_THETA;
};
