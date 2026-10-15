// Keyboard teleoperation with recording, for the real Neato or the simulator.
//
// Every scan, wheel reading, motor command and bumper event is written to a JSON Lines file
// (format in include/datalog/drive_log.hpp). These recordings are the input for calibration,
// scan matching and mapping later on, so the recorder runs whenever the robot is driven.
//
// Driving works in short segments: each key press sends a SetMotor command for 100 mm (or a
// 15 degree turn). Holding the key lets the terminal's key repeat re-send the command before
// the segment ends, so the robot moves continuously. Releasing the key stops it within one
// segment. This acts as a simple dead-man switch without needing key-release events.
//
// Safety:
//  - a pressed bumper stops the robot, backs it off 60 mm and blocks forward driving for 1 s
//  - Ctrl+C, q, or any exit path stops the wheels, stops the LDS and leaves test mode
//
// Usage:
//   teleop [port] [--log file.jsonl]                    real robot (default /dev/ttyACM0, COM9)
//   teleop --sim [map file] [--log file.jsonl]          simulated robot in the apartment
//
// Keys: w/s forward/back, a/d turn left/right, space or x stop, +/- speed, q quit
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "core/geometry.hpp"
#include "core/odometry.hpp"
#include "datalog/drive_log.hpp"
#include "neato/real_robot.hpp"
#include "sim/sim_robot.hpp"
#include "sim/world.hpp"
#include "tools/keyboard.hpp"

namespace {

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop = true; }

constexpr double kSegmentMm = 100.0;       // forward/backward segment per key press
constexpr double kTurnDeg = 15.0;          // turn per key press
constexpr double kTurnSpeedMmS = 100.0;    // wheel speed while turning
constexpr double kBackoffMm = 60.0;        // reverse after a bumper hit
constexpr double kResendS = 0.15;          // do not flood the serial link with repeated keys
constexpr double kScanPeriodS = 0.2;       // LDS turns at ~5 Hz, polling faster gives duplicates
constexpr double kBatteryPeriodS = 5.0;
constexpr double kSimStepS = 0.02;

std::string defaultLogPath() {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &now);
#else
    localtime_r(&now, &tm);
#endif
    std::ostringstream s;
    s << "logs/drive_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".jsonl";
    return s.str();
}

void printHelp() {
    std::cout << "  w/s  forward/back      a/d  turn left/right\n"
                 "  space or x  stop       +/-  speed\n"
                 "  q  quit (Ctrl+C works too)\n\n";
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    std::string port = "COM9";
#else
    std::string port = "/dev/ttyACM0";
#endif
    bool use_sim = false;
    std::string map_path = "maps/apartment.map";
    std::string log_path = defaultLogPath();

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--sim") {
            use_sim = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') map_path = argv[++i];
        } else if (a == "--log" && i + 1 < argc) {
            log_path = argv[++i];
        } else if (a == "-h" || a == "--help") {
            std::cout << "Usage: teleop [port] [--log file]\n       teleop --sim [map] [--log file]\n";
            printHelp();
            return 0;
        } else {
            port = a;
        }
    }

    // ---- Robot backend -----------------------------------------------------------------------
    sim::World world;
    std::unique_ptr<sim::SimRobot> sim_robot;
    neato::RealRobot real_robot;
    neato::IRobot* robot = nullptr;
    datalog::LogHeader header;

    if (use_sim) {
        std::string err;
        if (!world.load(map_path, err)) {
            std::cerr << "Cannot load map " << map_path << ": " << err << "\n";
            return 1;
        }
        sim_robot = std::make_unique<sim::SimRobot>(world, world.start().value_or(geo::Pose2D{}));
        robot = sim_robot.get();
        header.source = "sim:" + map_path;
        header.wheelbase_m = sim_robot->params().wheelbase_m;
        header.lds_offset_m = sim_robot->params().lds_offset_m;
    } else {
        if (!real_robot.connect(port)) {
            std::cerr << "Cannot open " << port << ": " << real_robot.lastError() << "\n";
            return 1;
        }
        robot = &real_robot;
        header.source = "real:" + port;
    }

    // ---- Recorder ----------------------------------------------------------------------------
    const auto parent = std::filesystem::path(log_path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    datalog::DriveLogWriter log;
    if (!log.open(log_path, header)) {
        std::cerr << "Cannot write " << log_path << "\n";
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "Connected to " << header.source << ", recording to " << log_path << "\n";
    printHelp();

    if (!robot->enableMotion(true)) {
        std::cerr << "Could not enable motion (TestMode / LDS)\n";
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    auto now = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };

    core::Odometry odom(header.wheelbase_m);
    double speed = 150.0;  // mm/s
    char last_key = 0;
    double last_send = -1.0, last_scan = -1.0, last_battery = -kBatteryPeriodS, sim_time = 0.0;
    double forward_blocked_until = 0.0;
    bool bumped = false;
    std::string battery_text = "--.- V";
    long scans = 0;

    auto drive = [&](double l, double r, double v) {
        robot->driveWheels(l, r, v);
        log.command(now(), l, r, v);
        last_send = now();
    };
    auto stop = [&] { drive(0.0, 0.0, speed); };

    {
        tools::RawKeyboard kb;  // restores the terminal when this scope ends
        bool running = true;
        while (running && !g_stop) {
            // Keep only the newest key: with key repeat several may be queued
            int key = -1;
            for (int k = kb.poll(); k != -1; k = kb.poll()) key = k;

            if (key != -1) {
                const double t = now();
                const bool repeat = (key == last_key) && (t - last_send < kResendS);
                const double turn_mm = 0.5 * header.wheelbase_m * geo::deg2rad(kTurnDeg) * 1000.0;
                switch (key) {
                    case 'w': case 'W':
                        if (t < forward_blocked_until) break;
                        if (!repeat) drive(kSegmentMm, kSegmentMm, speed);
                        break;
                    case 's': case 'S':
                        if (!repeat) drive(-kSegmentMm, -kSegmentMm, speed);
                        break;
                    case 'a': case 'A':
                        if (!repeat) drive(-turn_mm, turn_mm, kTurnSpeedMmS);
                        break;
                    case 'd': case 'D':
                        if (!repeat) drive(turn_mm, -turn_mm, kTurnSpeedMmS);
                        break;
                    case ' ': case 'x': case 'X':
                        stop();
                        log.event(t, "stop key");
                        break;
                    case '+': case '=':
                        speed = std::min(300.0, speed + 50.0);
                        break;
                    case '-': case '_':
                        speed = std::max(50.0, speed - 50.0);
                        break;
                    case 'q': case 'Q':
                        running = false;
                        break;
                    default:
                        break;
                }
                last_key = static_cast<char>(key);
            }

            // Simulator follows wall-clock time so driving feels the same as on the robot
            if (sim_robot) {
                const double target = now();
                while (sim_time + kSimStepS <= target) {
                    sim_robot->step(kSimStepS);
                    sim_time += kSimStepS;
                }
            }

            // Sensors
            if (auto w = robot->getWheelPositions()) {
                odom.update(*w);
                log.odom(now(), *w);
            }
            if (auto b = robot->getBumpers()) {
                if (b->any() && !bumped) {
                    stop();
                    log.event(now(), "bumper");
                    drive(-kBackoffMm, -kBackoffMm, 100.0);
                    forward_blocked_until = now() + 1.0;
                }
                bumped = b->any();
            }
            if (now() - last_scan >= kScanPeriodS) {
                last_scan = now();
                if (auto s = robot->getScan()) {
                    log.scan(now(), *s);
                    ++scans;
                }
            }
            if (now() - last_battery >= kBatteryPeriodS) {
                last_battery = now();
                if (auto b = robot->getBattery()) {
                    std::ostringstream s;
                    s << std::fixed << std::setprecision(2) << b->voltage_v << " V";
                    battery_text = s.str();
                }
            }

            const auto& p = odom.pose();
            std::printf("\r speed %3.0f mm/s | odom x %6.2f m  y %6.2f m  th %6.1f deg | battery %s | scans %ld %s   ",
                        speed, p.x, p.y, geo::rad2deg(p.theta), battery_text.c_str(), scans,
                        bumped ? "| BUMPER" : "");
            std::fflush(stdout);

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    // ---- Shutdown: always leave the robot stopped and out of test mode ----------------------
    stop();
    robot->enableMotion(false);
    log.event(now(), "shutdown");
    std::cout << "\nStopped. " << log.lines() << " lines written to " << log_path << "\n";
    return 0;
}
