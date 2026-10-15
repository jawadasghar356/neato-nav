// Experiment: which serial command actually stops the Neato while it is driving?
//
// Background: teleop sends "SetMotor LWheelDist 0 RWheelDist 0 Speed 150" as stop. The first
// recorded drive (logs of 2026-10-15) shows the robot ignores it and finishes the current
// 100 mm segment. A new non-zero SetMotor, on the other hand, replaces the running one
// (the bumper back-off works). So the firmware accepts replacements but not a zero distance.
//
// For each candidate stop command the robot drives forward (300 mm at 150 mm/s). After about
// 80 mm the candidate is sent, and the wheel encoders show how much further the robot rolls
// and how long it takes to stand still. The robot then drives back to where it started, so
// the whole test needs about 40 cm of free floor in front of it.
//
// Usage: stop_test [port]        robot off the dock, at least 40 cm free in front
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "neato/real_robot.hpp"

namespace {

using Clock = std::chrono::steady_clock;
std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop = true; }

struct Candidate {
    std::string name;
    std::vector<std::string> commands;  // sent one after another as "the stop"
    std::vector<std::string> after;     // sent once the robot stands still (re-enable etc.)
};

double secondsSince(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

// Mean travel of both wheels in mm, or NaN if the robot did not answer
double meanTravel(neato::RealRobot& robot) {
    auto w = robot.getWheelPositions();
    return w ? 0.5 * (w->left_mm + w->right_mm) : std::nan("");
}

// Polls the encoders until they have not changed for `quiet_s`. Returns the final position.
double waitUntilStill(neato::RealRobot& robot, double quiet_s, double timeout_s, double& still_at_s,
                      Clock::time_point t0) {
    double last = meanTravel(robot);
    auto last_change = Clock::now();
    still_at_s = secondsSince(t0);
    while (!g_stop && secondsSince(last_change) < quiet_s && secondsSince(t0) < timeout_s) {
        const double now = meanTravel(robot);
        if (!std::isnan(now) && std::fabs(now - last) >= 1.0) {
            last = now;
            last_change = Clock::now();
            still_at_s = secondsSince(t0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return last;
}

std::string oneLine(std::string s) {
    for (char& c : s)
        if (c == '\n' || c == '\r' || c == '\x1A') c = ' ';
    while (!s.empty() && s.back() == ' ') s.pop_back();
    return s.empty() ? "(no answer)" : s;
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    std::string port = argc > 1 ? argv[1] : "COM9";
#else
    std::string port = argc > 1 ? argv[1] : "/dev/ttyACM0";
#endif

    const std::vector<Candidate> candidates = {
        {"A  zero distance (current teleop stop)", {"SetMotor LWheelDist 0 RWheelDist 0 Speed 150"}, {}},
        {"B  1 mm forward, replaces running move", {"SetMotor LWheelDist 1 RWheelDist 1 Speed 150"}, {}},
        {"C  disable wheel motors, then enable",
         {"SetMotor LWheelDisable RWheelDisable"},
         {"SetMotor LWheelEnable RWheelEnable"}},
        {"D  zero distance with zero speed", {"SetMotor LWheelDist 0 RWheelDist 0 Speed 0"}, {}},
    };

    neato::RealRobot robot;
    if (!robot.connect(port)) {
        std::cerr << "Cannot open " << port << ": " << robot.lastError() << "\n";
        return 1;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "Stop command test on " << port << ". The robot drives about 10 cm forward and back,\n"
              << "four times. Keep 40 cm free in front of it. Ctrl+C aborts.\n\n";
    robot.raw("TestMode On");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    struct Result { std::string name, answer; double at_cmd, extra, stop_time; bool ok; };
    std::vector<Result> results;

    for (const auto& c : candidates) {
        if (g_stop) break;
        Result r{c.name, "", 0, 0, 0, false};
        const double start = meanTravel(robot);
        if (std::isnan(start)) { std::cerr << "No encoder answer, aborting\n"; break; }

        const auto t0 = Clock::now();
        robot.raw("SetMotor LWheelDist 300 RWheelDist 300 Speed 150");

        // Wait until the robot has rolled about 80 mm
        double pos = start;
        while (!g_stop && pos - start < 80.0 && secondsSince(t0) < 3.0) {
            const double p = meanTravel(robot);
            if (!std::isnan(p)) pos = p;
        }
        if (g_stop || pos - start < 80.0) {
            std::cerr << c.name << ": robot did not start moving, aborting\n";
            break;
        }

        // The candidate stop
        const auto t_stop = Clock::now();
        for (const auto& cmd : c.commands) r.answer += oneLine(robot.raw(cmd)) + " ";
        r.at_cmd = meanTravel(robot) - start;

        double still_at = 0.0;
        const double end = waitUntilStill(robot, 0.5, 6.0, still_at, t_stop);
        r.extra = (end - start) - r.at_cmd;
        r.stop_time = still_at;
        r.ok = true;
        for (const auto& cmd : c.after) robot.raw(cmd);
        results.push_back(r);
        std::printf("%-42s  moved %5.0f mm after the stop, still after %.2f s\n", c.name.c_str(), r.extra,
                    r.stop_time);

        // Drive back to the start position
        const double back = start - end;
        if (std::fabs(back) > 5.0) {
            robot.raw("SetMotor LWheelDist " + std::to_string(std::lround(back)) + " RWheelDist " +
                      std::to_string(std::lround(back)) + " Speed 150");
            double dummy = 0.0;
            waitUntilStill(robot, 0.5, 6.0, dummy, Clock::now());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Leave the robot safe whatever happened above
    robot.raw("SetMotor LWheelDisable RWheelDisable");
    robot.raw("SetMotor LWheelEnable RWheelEnable");
    robot.raw("TestMode Off");

    std::cout << "\nResult (a full 300 mm move minus ~80 mm means the stop was ignored: ~220 mm)\n";
    for (const auto& r : results)
        std::printf("  %-42s %5.0f mm  %.2f s  answer: %s\n", r.name.c_str(), r.extra, r.stop_time,
                    r.answer.c_str());
    std::cout << "\nPaste this result into the chat.\n";
    return 0;
}
