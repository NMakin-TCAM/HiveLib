#include "hivelib/api.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>

static double rand01() { return (double)std::rand() / (double)RAND_MAX; }

// Box–Muller normal noise
static double randn(double sigma) {
    double u1 = std::max(1e-9, rand01());
    double u2 = rand01();
    double z0 = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
    return z0 * sigma;
}

MonteCarloLocalizer::MonteCarloLocalizer(
    pros::Rotation* rot, // replace these with your defined one
    pros::IMU* imu, // replace these with your defined one
    const DistSensorMount* mounts,
    int mountCount,
    double trackingWheelDiamIn
)
: m_vertRot(vertOdomRot),
  m_imu(imu),
  m_mounts(mounts),
  m_mountCount(mountCount),
  m_trackWheelDiamIn(trackingWheelDiamIn) {

    reset(mclcfg::INIT_X, mclcfg::INIT_Y, mclcfg::INIT_THETA);
}

void MonteCarloLocalizer::start() {
    if (m_running) return;
    m_running = true;
    m_task = new pros::Task([this]{ this->loop_(); }, "MCL");
}

void MonteCarloLocalizer::stop() {
    m_running = false;
    if (m_task) { delete m_task; m_task = nullptr; }
}

void MonteCarloLocalizer::reset(double x, double y, double thetaRad) {
    // Publish reset pose
    m_poseMutex.take(TIMEOUT_MAX);
    m_pose = lemlib::Pose{(float)x, (float)y, (float)thetaRad};
    m_conf = 0.0;
    m_poseMutex.give();

    // initialize particles around start
    for (int i = 0; i < mclcfg::N_PARTICLES; i++) {
        m_particles[i].x = x + randn(2.0);
        m_particles[i].y = y + randn(2.0);
        m_particles[i].th = wrapAngle_(thetaRad + randn(0.15));
        m_particles[i].w = 1.0 / (double)mclcfg::N_PARTICLES;
    }

    // initialize odom refs
    m_lastVertDeg = m_vertRot ? m_vertRot->get_position() : 0.0;
    m_lastImuRad = m_imu ? (m_imu->get_rotation() * M_PI / 180.0) : thetaRad;

    m_meanX = x;
    m_meanY = y;
    m_meanTh = thetaRad;
}

lemlib::Pose MonteCarloLocalizer::getPose() const {
    m_poseMutex.take(TIMEOUT_MAX);
    lemlib::Pose p = m_pose;
    m_poseMutex.give();
    return p;
}

double MonteCarloLocalizer::confidence() const {
    m_poseMutex.take(TIMEOUT_MAX);
    double c = m_conf;
    m_poseMutex.give();
    return c;
}

void MonteCarloLocalizer::loop_() {
    int tick = 0;
    while (m_running) {
        // Read deltas
        const double vertDeg = m_vertRot->get_position();
        const double dVertDeg = vertDeg - m_lastVertDeg;
        m_lastVertDeg = vertDeg;

        // Convert tracking wheel degrees -> inches traveled along robot forward axis
        const double wheelCirc = M_PI * m_trackWheelDiamIn;
        const double ds = (dVertDeg / 360.0) * wheelCirc;

        const double imuRad = m_imu->get_rotation() * M_PI / 180.0;
        double dth = wrapAngle_(imuRad - m_lastImuRad);
        m_lastImuRad = imuRad;

        // Predict every loop (smooth)
        predict_(ds, dth);

        // Do measurement update at a lower rate
        tick++;
        const bool doMeasTick = (tick % mclcfg::MEAS_UPDATE_EVERY_N == 0);
        const bool movedEnough = (std::fabs(ds) >= mclcfg::RESAMPLE_MIN_DS_IN) ||
                                (std::fabs(dth) >= mclcfg::RESAMPLE_MIN_DTH_RAD);

        if (m_mountCount > 0 && doMeasTick && movedEnough) {
            measurementUpdate_();
            resample_();
        }

        // Publish mean pose
        double mx = 0, my = 0;
        double c = 0, s = 0;
        for (int i = 0; i < mclcfg::N_PARTICLES; i++) {
            mx += m_particles[i].x * m_particles[i].w;
            my += m_particles[i].y * m_particles[i].w;
            c  += std::cos(m_particles[i].th) * m_particles[i].w;
            s  += std::sin(m_particles[i].th) * m_particles[i].w;
        }
        const double mth = std::atan2(s, c);

        m_meanX = mx; m_meanY = my; m_meanTh = mth;

        double vx = 0.0, vy = 0.0;
        for (int i = 0; i < mclcfg::N_PARTICLES; i++) {
            const double dx = m_particles[i].x - mx;
            const double dy = m_particles[i].y - my;
            vx += dx * dx * m_particles[i].w;
            vy += dy * dy * m_particles[i].w;
        }
        const double stdxy = std::sqrt(vx + vy); // overall position spread in inches

        // Confidence heuristic: effective sample size (Neff)
        double sumsq = 0;
        for (int i = 0; i < mclcfg::N_PARTICLES; i++) sumsq += m_particles[i].w * m_particles[i].w;
        const double neff = (sumsq > 1e-9) ? (1.0 / sumsq) : 0.0;
        double conf = 1.0 - (stdxy - mclcfg::CONF_GOOD_STD_IN) / (mclcfg::CONF_BAD_STD_IN - mclcfg::CONF_GOOD_STD_IN);
        conf = std::min(1.0, std::max(0.0, conf));

        m_poseMutex.take(TIMEOUT_MAX);
        const double mth_deg = mth * 180.0 / M_PI;
        m_pose = lemlib::Pose{(float)mx, (float)my, (float)mth_deg};
        m_conf = conf;
        m_poseMutex.give();

        pros::delay(mclcfg::UPDATE_MS);
    }
}

void MonteCarloLocalizer::predict_(double ds, double dth) {
    for (int i = 0; i < mclcfg::N_PARTICLES; i++) {
        const double nds  = ds  + randn(mclcfg::SIGMA_TRANS_IN);
        const double ndth = dth + randn(mclcfg::SIGMA_THETA_RAD);

        m_particles[i].th = wrapAngle_(m_particles[i].th + ndth);
        m_particles[i].x += nds * std::cos(m_particles[i].th);
        m_particles[i].y += nds * std::sin(m_particles[i].th);

        const bool oob =
            (m_particles[i].x < mclcfg::FIELD_MIN_X) || (m_particles[i].x > mclcfg::FIELD_MAX_X) ||
            (m_particles[i].y < mclcfg::FIELD_MIN_Y) || (m_particles[i].y > mclcfg::FIELD_MAX_Y);

        if (oob) {
            m_particles[i].x  = m_meanX + randn(mclcfg::OOB_RESPAWN_XY_SIGMA_IN);
            m_particles[i].y  = m_meanY + randn(mclcfg::OOB_RESPAWN_XY_SIGMA_IN);
            m_particles[i].th = wrapAngle_(m_meanTh + randn(mclcfg::OOB_RESPAWN_TH_SIGMA_RAD));
            m_particles[i].w  = 1.0 / (double)mclcfg::N_PARTICLES;
        }
    }
}

void MonteCarloLocalizer::measurementUpdate_() {
    // First: compute log-weights (to avoid underflow)
    double logw[mclcfg::N_PARTICLES];

    // Small epsilon to prevent log(0)
    constexpr double EPS = 1e-12;

    auto expectedFromPose = [&](double px, double py, double pth, const DistSensorMount& m, double& outExp) {
        const double sx = px + (m.x_off * std::cos(pth) - m.y_off * std::sin(pth));
        const double sy = py + (m.x_off * std::sin(pth) + m.y_off * std::cos(pth));
        const double rayTh = wrapAngle_(pth + m.yaw_off);
        return rayRectDist_(sx, sy, std::cos(rayTh), std::sin(rayTh),
                            mclcfg::FIELD_MIN_X, mclcfg::FIELD_MIN_Y,
                            mclcfg::FIELD_MAX_X, mclcfg::FIELD_MAX_Y, outExp);
    };

    for (int i = 0; i < mclcfg::N_PARTICLES; i++) {
        double lw = 0.0;

        for (int k = 0; k < m_mountCount; k++) {

            
            const auto& m = m_mounts[k];
            if (!m.sensor) continue;

            const int mm = m.sensor->get_distance();
            if (mm < mclcfg::DIST_MIN_MM || mm > mclcfg::DIST_MAX_MM) continue;

            const double z = mm / 25.4; // inches

            double expMean = 0.0;
            if (expectedFromPose(m_meanX, m_meanY, m_meanTh, m, expMean)) {
                if (std::fabs(z - expMean) > mclcfg::DIST_GATE_IN) {
                    continue; // likely hit a game object, ignore
                }
            }

            // sensor world origin
            const double th = m_particles[i].th;
            const double sx = m_particles[i].x + (m.x_off * std::cos(th) - m.y_off * std::sin(th));
            const double sy = m_particles[i].y + (m.x_off * std::sin(th) + m.y_off * std::cos(th));

            // sensor ray direction in world
            const double rayTh = wrapAngle_(th + m.yaw_off);
            const double dx = std::cos(rayTh);
            const double dy = std::sin(rayTh);

            double expected = 0.0;
            if (!rayRectDist_(sx, sy, dx, dy,
                              mclcfg::FIELD_MIN_X, mclcfg::FIELD_MIN_Y,
                              mclcfg::FIELD_MAX_X, mclcfg::FIELD_MAX_Y,
                              expected)) {
                continue;
            }

            const double err = (z - expected);
            const double p = gaussProb_(err, mclcfg::SIGMA_DIST_IN);
            lw += std::log(p + EPS);
        }

        logw[i] = lw;
    }

    // Convert log-weights -> weights safely by subtracting max logw
    double maxlw = logw[0];
    for (int i = 1; i < mclcfg::N_PARTICLES; i++) maxlw = std::max(maxlw, logw[i]);

    double sum = 0.0;
    for (int i = 0; i < mclcfg::N_PARTICLES; i++) {
        const double w = std::exp(logw[i] - maxlw);
        m_particles[i].w = w;
        sum += w;
    }

    // Normalize
    if (sum < 1e-12) {
        const double u = 1.0 / (double)mclcfg::N_PARTICLES;
        for (int i = 0; i < mclcfg::N_PARTICLES; i++) m_particles[i].w = u;
        return;
    }
    for (int i = 0; i < mclcfg::N_PARTICLES; i++) m_particles[i].w /= sum;
}

void MonteCarloLocalizer::resample_() {
    // Systematic resampling
    Particle newP[mclcfg::N_PARTICLES];
    const double step = 1.0 / (double)mclcfg::N_PARTICLES;
    double r = rand01() * step;
    double c = m_particles[0].w;
    int i = 0;

    for (int m = 0; m < mclcfg::N_PARTICLES; m++) {
        const double u = r + m * step;
        while (u > c && i < mclcfg::N_PARTICLES - 1) {
            i++;
            c += m_particles[i].w;
        }
        newP[m] = m_particles[i];
        newP[m].w = step; // uniform after resample
    }

    for (int j = 0; j < mclcfg::N_PARTICLES; j++) m_particles[j] = newP[j];
}

double MonteCarloLocalizer::wrapAngle_(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double MonteCarloLocalizer::gaussProb_(double err, double sigma) {
    const double s2 = sigma * sigma;
    return std::exp(-0.5 * (err * err) / (s2 + 1e-9));
}

// Ray from (ox,oy) along (dx,dy) to rectangle; returns nearest positive distance
bool MonteCarloLocalizer::rayRectDist_(double ox, double oy, double dx, double dy,
                                      double xmin, double ymin, double xmax, double ymax,
                                      double& outDist) {
    // Intersect with 4 lines; take smallest t>0
    double best = 1e18;
    bool hit = false;

    auto tryHit = [&](double t) {
        if (t > 0 && t < best) { best = t; hit = true; }
    };

    // x = xmin/xmax
    if (std::fabs(dx) > 1e-9) {
        double t = (xmin - ox) / dx;
        double y = oy + t * dy;
        if (y >= ymin && y <= ymax) tryHit(t);

        t = (xmax - ox) / dx;
        y = oy + t * dy;
        if (y >= ymin && y <= ymax) tryHit(t);
    }

    // y = ymin/ymax
    if (std::fabs(dy) > 1e-9) {
        double t = (ymin - oy) / dy;
        double x = ox + t * dx;
        if (x >= xmin && x <= xmax) tryHit(t);

        t = (ymax - oy) / dy;
        x = ox + t * dx;
        if (x >= xmin && x <= xmax) tryHit(t);
    }

    if (!hit) return false;
    outDist = best;
    return true;
}