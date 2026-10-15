#include "sim/sim_robot.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

SimRobot::SimRobot(const World& world, geo::Pose2D start, SimParams params, unsigned seed)
    : world_(world), pose_(start), p_(params), rng_(seed), scan_buf_(360) {}

geo::Pose2D SimRobot::lidarPose() const {
    return {pose_.x + p_.lds_offset_m * std::cos(pose_.theta), pose_.y + p_.lds_offset_m * std::sin(pose_.theta),
            pose_.theta};
}

// D shape in the robot frame: flat front at x = front_m, half circle at the back
std::vector<geo::Point2D> SimRobot::footprint(const geo::Pose2D& p) const {
    std::vector<geo::Point2D> local = {{p_.front_m, p_.half_width_m}};
    for (int deg = 90; deg <= 270; deg += 20)
        local.push_back({p_.half_width_m * std::cos(geo::deg2rad(deg)), p_.half_width_m * std::sin(geo::deg2rad(deg))});
    local.push_back({p_.front_m, -p_.half_width_m});
    std::vector<geo::Point2D> world;
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    for (const auto& q : local) world.push_back({p.x + c * q.x - s * q.y, p.y + s * q.x + c * q.y});
    return world;
}

bool SimRobot::driveWheels(double left_mm, double right_mm, double speed_mm_s) {
    // Like the Neato: the wheel with the longer distance runs at `speed`,
    // the other one proportionally slower, so both finish at the same time.
    remaining_left_m_ = left_mm / 1000.0;
    remaining_right_m_ = right_mm / 1000.0;
    const double longest = std::max(std::fabs(remaining_left_m_), std::fabs(remaining_right_m_));
    if (longest < 1e-9) {
        speed_left_ = speed_right_ = 0.0;
        return true;
    }
    const double v = speed_mm_s / 1000.0;
    speed_left_ = v * remaining_left_m_ / longest;
    speed_right_ = v * remaining_right_m_ / longest;
    return true;
}

bool SimRobot::isMoving() const {
    return std::fabs(remaining_left_m_) > 1e-6 || std::fabs(remaining_right_m_) > 1e-6;
}

void SimRobot::step(double dt) {
    auto advance = [dt](double& remaining, double speed) {
        double d = speed * dt;
        if (std::fabs(d) > std::fabs(remaining)) d = remaining;  // do not overshoot
        remaining -= d;
        return d;
    };
    const double dl = advance(remaining_left_m_, speed_left_);
    const double dr = advance(remaining_right_m_, speed_right_);

    if (dl != 0.0 || dr != 0.0) {
        // Where the wheels would take the robot
        geo::Pose2D next = pose_;
        const double ds = 0.5 * (dl + dr);
        const double dtheta = (dr - dl) / p_.wheelbase_m;
        const double mid = pose_.theta + 0.5 * dtheta;
        next.x += ds * std::cos(mid);
        next.y += ds * std::sin(mid);
        next.theta = geo::wrapAngle(pose_.theta + dtheta);

        geo::Point2D contact;
        if (world_.collidesPolygon(footprint(next), &contact)) {
            // Blocked: the robot stays, the bumper is pressed, the wheels slip for this step
            const double bearing =
                geo::wrapAngle(std::atan2(contact.y - pose_.y, contact.x - pose_.x) - pose_.theta);
            const bool front = std::fabs(bearing) < geo::deg2rad(60);
            bumpers_ = {};
            if (bearing >= 0) (front ? bumpers_.left_front : bumpers_.left_side) = true;
            else (front ? bumpers_.right_front : bumpers_.right_side) = true;
            remaining_left_m_ = remaining_right_m_ = 0.0;
            speed_left_ = speed_right_ = 0.0;
        } else {
            pose_ = next;
            bumpers_ = {};
        }
        // Encoders count wheel rotation, even when the wheels slip
        std::normal_distribution<double> noise(0.0, p_.enc_noise_mm);
        enc_left_mm_ += dl * 1000.0 * p_.enc_scale_left + noise(rng_);
        enc_right_mm_ += dr * 1000.0 * p_.enc_scale_right + noise(rng_);
    }

    // The LDS turret keeps spinning: measure the beams it passes during this step
    beam_acc_ += 360.0 * p_.lds_rate_hz * dt;
    while (beam_acc_ >= 1.0) {
        beam_acc_ -= 1.0;
        bool ghost = false;
        scan_buf_[static_cast<size_t>(next_deg_)] = measureBeam(next_deg_, ghost);
        if (ghost && scan_buf_[static_cast<size_t>(next_deg_)].valid) ++ghosts_buf_;
        if (++next_deg_ == 360) {  // one full turn finished
            next_deg_ = 0;
            last_scan_ = scan_buf_;
            ghosts_last_ = ghosts_buf_;
            ghosts_buf_ = 0;
            ++scan_count_;
        }
    }
}

neato::LaserReading SimRobot::measureBeam(int deg, bool& ghost) {
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    neato::LaserReading r;
    r.angle_deg = deg;

    const geo::Pose2D lp = lidarPose();
    double dir = lp.theta + geo::deg2rad(deg);
    auto hit = world_.raycast({lp.x, lp.y}, dir, p_.lds_max_m);
    if (!hit) return r;

    double total = hit->distance;
    double loss = 1.0;
    const Material* m = &world_.material(hit->material);
    double dx = std::cos(dir), dy = std::sin(dir);
    double cos_inc = std::fabs(dx * hit->normal.x + dy * hit->normal.y);

    // Glossy or glass surface: the beam may bounce off like a mirror and come back
    // from somewhere else. The LDS then reports the full path length -> ghost point.
    if (m->mirror > 0.0 && uni(rng_) < m->mirror) {
        const double dn = dx * hit->normal.x + dy * hit->normal.y;
        const double rx = dx - 2.0 * dn * hit->normal.x, ry = dy - 2.0 * dn * hit->normal.y;
        dir = std::atan2(ry, rx);
        const geo::Point2D from{hit->point.x + rx * 1e-4, hit->point.y + ry * 1e-4};
        auto hit2 = world_.raycast(from, dir, p_.lds_max_m - total);
        if (!hit2) return r;
        total += hit2->distance;
        m = &world_.material(hit2->material);
        cos_inc = std::fabs(rx * hit2->normal.x + ry * hit2->normal.y);
        loss = 0.7;
        ghost = true;
    }
    if (total < p_.lds_min_m || total > p_.lds_max_m) return r;

    // Returned signal: surface reflectivity x angle of incidence / distance^2
    const double signal = p_.lds_gain * m->reflectivity * cos_inc * loss / std::max(total * total, 0.09);
    double p_detect = std::clamp((signal - 0.7) / 0.6, 0.0, 1.0);  // soft detection threshold
    if (cos_inc < 0.12) p_detect = 0.0;                           // grazing angle > ~83 degrees
    if (uni(rng_) > p_detect || uni(rng_) < p_.lds_random_dropout) return r;

    // Triangulation LDS: error grows with distance squared; reported in whole millimetres
    std::normal_distribution<double> noise(0.0, p_.lds_noise_a + p_.lds_noise_b * total * total);
    r.range_m = std::round((total + noise(rng_)) * 1000.0) / 1000.0;
    r.intensity = static_cast<int>(std::min(4000.0, 300.0 * signal));
    r.valid = r.range_m > 0.0;
    return r;
}

std::optional<neato::LaserScan> SimRobot::getScan() {
    if (last_scan_.empty()) return std::nullopt;
    neato::LaserScan scan;
    scan.readings = last_scan_;
    scan.rotation_hz = p_.lds_rate_hz;
    return scan;
}

// The real robot reports whole millimetres
std::optional<neato::WheelPositions> SimRobot::getWheelPositions() {
    return neato::WheelPositions{std::round(enc_left_mm_), std::round(enc_right_mm_)};
}

std::optional<neato::BatteryStatus> SimRobot::getBattery() {
    neato::BatteryStatus b;
    b.voltage_v = 15.8;
    b.fuel_percent = 90;
    return b;
}

}  // namespace sim
