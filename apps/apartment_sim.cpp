// Experiment 1 in the apartment: "How far can the robot trust its wheel odometry?"
//
// The simulated Neato leaves its dock, drives the mission route from the map file
// (to the entrance door and back) and returns to the dock, several times in a row.
// It navigates ONLY with its own odometry, exactly like a robot without LiDAR-based
// correction. Every LDS turn is recorded together with the true pose and the robot's
// belief, and written to a JSON file that viewer/index.html plays back.
//
// Usage: apartment_sim [map file] [output json] [missions]
//        defaults: maps/apartment.map  sim_log.json  2
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "core/geometry.hpp"
#include "core/odometry.hpp"
#include "sim/sim_robot.hpp"
#include "sim/world.hpp"

namespace {

struct Frame {
    double t;
    geo::Pose2D truth, belief;
    std::vector<int> ranges_cm;  // every 2nd degree, 0 = no reading
    bool bump;
    int ghosts;
};

std::string pose3(const geo::Pose2D& p) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(3) << '[' << p.x << ',' << p.y << ',' << p.theta << ']';
    return s.str();
}

void writeJson(const std::string& path, const sim::World& w, const sim::SimParams& prm,
               const std::vector<Frame>& frames, double distance, double err, double head_err, int bumps,
               const std::vector<bool>& docked) {
    std::ofstream f(path);
    f << std::fixed << std::setprecision(3);
    f << "{\"meta\":{\"robot\":\"Neato Botvac D3 (simulated)\",\"duration\":" << frames.back().t
      << ",\"distance\":" << distance << ",\"final_error\":" << err << ",\"final_heading_error_deg\":" << head_err
      << ",\"bumps\":" << bumps << ",\"lds_offset\":" << prm.lds_offset_m << ",\"front\":" << prm.front_m
      << ",\"half_width\":" << prm.half_width_m << ",\"beam_step_deg\":2,\"lds_max\":" << prm.lds_max_m
      << ",\"encoder_mismatch_percent\":" << 100.0 * (prm.enc_scale_left - prm.enc_scale_right) << ",\"docked\":[";
    for (size_t i = 0; i < docked.size(); ++i) f << (i ? "," : "") << (docked[i] ? "true" : "false");
    f << "]},\n";

    f << "\"map\":{\"size\":[" << w.width() << ',' << w.height() << "],\"materials\":{";
    for (size_t i = 0; i < w.materials().size(); ++i) {
        const auto& m = w.materials()[i];
        f << (i ? "," : "") << '"' << m.name << "\":{\"color\":\"#" << m.color << "\",\"reflectivity\":"
          << m.reflectivity << ",\"mirror\":" << m.mirror << '}';
    }
    f << "},\"walls\":[";
    for (size_t i = 0; i < w.walls().size(); ++i) {
        const auto& s = w.walls()[i];
        f << (i ? "," : "") << '[' << s.seg.a.x << ',' << s.seg.a.y << ',' << s.seg.b.x << ',' << s.seg.b.y
          << ",\"" << w.material(s.material).name << "\"]";
    }
    f << "],\"boxes\":[";
    for (size_t i = 0; i < w.boxes().size(); ++i) {
        const auto& b = w.boxes()[i];
        f << (i ? "," : "") << "{\"name\":\"" << b.name << "\",\"x0\":" << b.x0 << ",\"y0\":" << b.y0
          << ",\"x1\":" << b.x1 << ",\"y1\":" << b.y1 << ",\"mat\":\"" << w.material(b.material).name << "\"}";
    }
    f << "],\"circles\":[";
    for (size_t i = 0; i < w.circles().size(); ++i) {
        const auto& c = w.circles()[i];
        f << (i ? "," : "") << "{\"name\":\"" << c.name << "\",\"cx\":" << c.c.x << ",\"cy\":" << c.c.y
          << ",\"r\":" << c.r << ",\"mat\":\"" << w.material(c.material).name << "\"}";
    }
    f << "]},\n\"route\":[";
    for (size_t i = 0; i < w.waypoints().size(); ++i)
        f << (i ? "," : "") << '[' << w.waypoints()[i].x << ',' << w.waypoints()[i].y << ']';
    f << "],\n\"frames\":[\n";
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& fr = frames[i];
        f << (i ? ",\n" : "") << "{\"t\":" << std::setprecision(2) << fr.t << std::setprecision(3)
          << ",\"tp\":" << pose3(fr.truth) << ",\"op\":" << pose3(fr.belief) << ",\"b\":" << (fr.bump ? 1 : 0)
          << ",\"g\":" << fr.ghosts << ",\"r\":[";
        for (size_t k = 0; k < fr.ranges_cm.size(); ++k) f << (k ? "," : "") << fr.ranges_cm[k];
        f << "]}";
    }
    f << "\n]}\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string map_path = argc > 1 ? argv[1] : "maps/apartment.map";
    const std::string out_path = argc > 2 ? argv[2] : "sim_log.json";
    const int missions = argc > 3 ? std::stoi(argv[3]) : 2;

    sim::World world;
    std::string error;
    if (!world.load(map_path, error) && !world.load("../" + map_path, error)) {
        std::cerr << error << "\n";
        return 1;
    }
    if (!world.start() || world.waypoints().empty()) {
        std::cerr << "The map needs a 'start' line and at least one 'waypoint'.\n";
        return 1;
    }

    const sim::SimParams params;
    sim::SimRobot robot(world, *world.start(), params);
    core::Odometry odom(params.wheelbase_m);
    odom.reset(*world.start());
    odom.update(*robot.getWheelPositions());

    const double dt = 0.02;
    double t = 0.0;
    int bumps = 0, last_scan = 0;
    bool bump_since_frame = false;
    std::vector<Frame> frames;
    std::vector<bool> dock_results;

    // Advance the world until the current motion command is finished.
    // Returns false if a bumper stopped the robot.
    auto run = [&]() {
        bool bumped = false;
        while (robot.isMoving()) {
            robot.step(dt);
            t += dt;
            odom.update(*robot.getWheelPositions());
            if (robot.getBumpers()->any() && !bumped) {
                bumped = true;
                bump_since_frame = true;
                ++bumps;
            }
            if (robot.scanCount() != last_scan) {  // a new LDS turn is complete: record a frame
                last_scan = robot.scanCount();
                Frame fr{t, robot.truePose(), odom.pose(), {}, bump_since_frame, robot.ghostsInLastScan()};
                const auto scan = robot.getScan();
                for (int d = 0; d < 360; d += 2) {
                    const auto& r = scan->readings[static_cast<size_t>(d)];
                    fr.ranges_cm.push_back(r.valid ? static_cast<int>(std::lround(r.range_m * 100.0)) : 0);
                }
                frames.push_back(fr);
                bump_since_frame = false;
            }
        }
        return !bumped;
    };

    // Mission: turn on the spot towards each waypoint, then drive straight to it,
    // using only the odometry pose. On a bump: back off 6 cm and continue.
    const double wb = params.wheelbase_m;
    for (int m = 0; m < missions; ++m) {
        for (const auto& wp : world.waypoints()) {
            const auto& o = odom.pose();
            const double turn = geo::wrapAngle(std::atan2(wp.y - o.y, wp.x - o.x) - o.theta);
            const double arc_mm = turn * wb / 2.0 * 1000.0;
            robot.driveWheels(-arc_mm, arc_mm, 120.0);
            if (!run()) { robot.driveWheels(-60, -60, 100); run(); continue; }

            const double dist_mm = std::hypot(wp.x - odom.pose().x, wp.y - odom.pose().y) * 1000.0;
            robot.driveWheels(dist_mm, dist_mm, 220.0);
            if (!run()) { robot.driveWheels(-60, -60, 100); run(); }
        }
        // Dock like the real Neato: face away from the dock, then reverse onto it.
        // If the turn is blocked (robot hit something) it gives up instead of guessing.
        const auto& st = *world.start();
        const double turn = geo::wrapAngle(st.theta - odom.pose().theta);
        robot.driveWheels(-turn * wb / 2.0 * 1000.0, turn * wb / 2.0 * 1000.0, 120.0);
        if (run()) {
            const auto& o = odom.pose();
            const double along = (st.x - o.x) * std::cos(o.theta) + (st.y - o.y) * std::sin(o.theta);
            robot.driveWheels(along * 1000.0, along * 1000.0, 120.0);  // negative = reverse
            run();
        }
        const auto& tp = robot.truePose();
        const double dock_err = std::hypot(tp.x - st.x, tp.y - st.y);
        const double dock_head = geo::rad2deg(std::fabs(geo::wrapAngle(tp.theta - st.theta)));
        const bool docked = dock_err < 0.05 && dock_head < 10.0;  // contacts must line up
        dock_results.push_back(docked);
        std::cout << "Mission " << m + 1 << ": " << (docked ? "docked" : "MISSED the dock") << " (off by "
                  << std::fixed << std::setprecision(2) << dock_err << " m, " << dock_head << " deg)\n";
        // Pause on the dock for a second between missions (LDS keeps spinning)
        for (int i = 0; i < 50; ++i) {
            robot.step(dt);
            t += dt;
            if (robot.scanCount() != last_scan) {
                last_scan = robot.scanCount();
                Frame fr{t, robot.truePose(), odom.pose(), {}, false, robot.ghostsInLastScan()};
                const auto scan = robot.getScan();
                for (int d = 0; d < 360; d += 2) {
                    const auto& r = scan->readings[static_cast<size_t>(d)];
                    fr.ranges_cm.push_back(r.valid ? static_cast<int>(std::lround(r.range_m * 100.0)) : 0);
                }
                frames.push_back(fr);
            }
        }
    }

    const auto& p = robot.truePose();
    const auto& o = odom.pose();
    const double err = std::hypot(p.x - o.x, p.y - o.y);
    const double head_err = geo::rad2deg(std::fabs(geo::wrapAngle(p.theta - o.theta)));
    const double dist = odom.distanceTravelled();

    std::cout << std::fixed << std::setprecision(2) << "Missions:              " << missions << "\n"
              << "Distance driven:       " << dist << " m in " << t << " s\n"
              << "Final position error:  " << err << " m (" << 100.0 * err / dist << " % of distance)\n"
              << "Final heading error:   " << head_err << " deg\n"
              << "Bumper hits:           " << bumps << "\n"
              << "LDS turns recorded:    " << frames.size() << "\n";
    writeJson(out_path, world, params, frames, dist, err, head_err, bumps, dock_results);
    std::cout << "Saved " << out_path << " - open viewer/index.html and load this file.\n";
    return 0;
}
