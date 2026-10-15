// Simulated Neato Botvac D3: same IRobot interface as the real robot.
//
// What is modelled, and why it matters for navigation research:
//  - D-shaped footprint (flat front, round back) for realistic collisions and bumpers
//  - differential drive with the Neato wheelbase
//  - wheel encoders with a small calibration error, noise, 1 mm resolution and
//    wheel slip when the robot is blocked  -> odometry drifts like on a real robot
//  - LDS (laser distance sensor) as on the real Neato:
//      * mounted behind the wheel axle, spinning at 5 Hz, 1 degree steps
//      * the 360 readings of one turn are taken at different moments, so a scan
//        taken while driving is distorted ("rolling shutter")
//      * triangulation sensor: noise grows with the square of the distance
//      * signal strength depends on surface colour, distance and angle of incidence:
//        black objects and walls hit at a flat angle often give no reading
//      * glossy and glass surfaces sometimes act as mirrors and create "ghost" points
// The true pose is available from the simulator only, as ground truth for evaluation.
#pragma once
#include <random>
#include <vector>

#include "core/geometry.hpp"
#include "neato/robot_interface.hpp"
#include "sim/world.hpp"

namespace sim {

struct SimParams {
    double wheelbase_m = 0.248;
    double front_m = 0.16;          // flat front edge, ahead of the wheel axle
    double half_width_m = 0.16;     // radius of the round back = half the robot width
    double lds_offset_m = -0.09;    // LDS turret sits behind the wheel axle
    double lds_rate_hz = 5.0;
    double lds_min_m = 0.15;
    double lds_max_m = 5.0;
    double lds_noise_a = 0.004;     // range noise std dev = a + b * r^2 (triangulation)
    double lds_noise_b = 0.006;
    double lds_gain = 40.0;         // signal margin of a white wall at 1 m, head-on
    double lds_random_dropout = 0.01;
    double enc_scale_left = 1.002;  // +0.2 % vs right wheel: typical calibration error
    double enc_scale_right = 1.000;
    double enc_noise_mm = 0.08;     // per simulation step (50 Hz)
};

class SimRobot : public neato::IRobot {
public:
    SimRobot(const World& world, geo::Pose2D start, SimParams params = {}, unsigned seed = 42);

    // Advance the simulation by dt seconds (the real robot moves on its own)
    void step(double dt);
    bool isMoving() const;

    const geo::Pose2D& truePose() const { return pose_; }
    geo::Pose2D lidarPose() const;
    std::vector<geo::Point2D> footprint(const geo::Pose2D& p) const;

    // Increases by one every time the LDS completes a turn
    int scanCount() const { return scan_count_; }
    int ghostsInLastScan() const { return ghosts_last_; }
    const SimParams& params() const { return p_; }

    bool enableMotion(bool) override { return true; }
    std::optional<neato::LaserScan> getScan() override;
    std::optional<neato::WheelPositions> getWheelPositions() override;
    bool driveWheels(double left_mm, double right_mm, double speed_mm_s) override;
    std::optional<neato::BatteryStatus> getBattery() override;
    std::optional<neato::Bumpers> getBumpers() override { return bumpers_; }

private:
    neato::LaserReading measureBeam(int deg, bool& ghost);

    const World& world_;
    geo::Pose2D pose_;
    SimParams p_;
    std::mt19937 rng_;

    double remaining_left_m_{0.0}, remaining_right_m_{0.0};
    double speed_left_{0.0}, speed_right_{0.0};  // m/s, signed
    double enc_left_mm_{0.0}, enc_right_mm_{0.0};
    neato::Bumpers bumpers_;

    double beam_acc_{0.0};  // fractional beams owed by the spinning turret
    int next_deg_{0};       // next degree the turret will measure
    std::vector<neato::LaserReading> scan_buf_, last_scan_;
    int ghosts_buf_{0}, ghosts_last_{0};
    int scan_count_{0};
};

}  // namespace sim
