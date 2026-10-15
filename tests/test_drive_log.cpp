// Tests for the JSON Lines drive recorder.
// Drives the simulated robot for a few seconds, records everything the way teleop does,
// then reads the file back and checks its structure. The file is kept so CI can plot it.
//
// Usage: test_drive_log [map file] [output.jsonl]
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "datalog/drive_log.hpp"
#include "sim/sim_robot.hpp"
#include "sim/world.hpp"

static int failures = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::cerr << "FAIL line " << __LINE__ << ": " #cond << "\n"; \
            ++failures;                                                   \
        }                                                                 \
    } while (0)

namespace {

// Returns the comma-separated integers of the JSON array stored under `key`
std::vector<int> intArray(const std::string& line, const std::string& key) {
    std::vector<int> out;
    const auto pos = line.find("\"" + key + "\":[");
    if (pos == std::string::npos) return out;
    const auto start = line.find('[', pos) + 1;
    const auto end = line.find(']', start);
    std::stringstream s(line.substr(start, end - start));
    std::string item;
    while (std::getline(s, item, ',')) out.push_back(std::stoi(item));
    return out;
}

std::string typeOf(const std::string& line) {
    const auto pos = line.find("\"type\":\"");
    if (pos == std::string::npos) return "";
    const auto start = pos + 8;
    return line.substr(start, line.find('"', start) - start);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string map_path = argc > 1 ? argv[1] : "maps/apartment.map";
    const std::string out_path = argc > 2 ? argv[2] : "drive_log_test.jsonl";

    // String escaping
    CHECK(datalog::jsonEscape("plain") == "plain");
    CHECK(datalog::jsonEscape("a\"b") == "a\\\"b");
    CHECK(datalog::jsonEscape("c:\\x\n") == "c:\\\\x\\n");

    // Scan encoding: index = angle, weak readings keep their range with flag 1
    {
        const std::string path = out_path + ".unit";
        datalog::DriveLogWriter w;
        CHECK(w.open(path, {}));
        neato::LaserScan scan;
        scan.rotation_hz = 5.02;
        neato::LaserReading valid;
        valid.angle_deg = 103; valid.range_m = 0.569; valid.intensity = 100; valid.valid = true;
        neato::LaserReading weak;
        weak.angle_deg = 102; weak.range_m = 0.570; weak.intensity = 46; weak.weak = true;
        neato::LaserReading none;
        none.angle_deg = 11;
        scan.readings = {none, weak, valid};
        w.scan(1.0, scan);
        CHECK(w.lines() == 2);

        std::ifstream in(path);
        std::string header, line;
        std::getline(in, header);
        std::getline(in, line);
        CHECK(typeOf(header) == "header");
        CHECK(typeOf(line) == "scan");
        const auto r = intArray(line, "r"), f = intArray(line, "f"), i = intArray(line, "i");
        CHECK(r.size() == 360 && f.size() == 360 && i.size() == 360);
        if (r.size() == 360 && f.size() == 360) {
            CHECK(r[103] == 569 && f[103] == 0 && i[103] == 100);
            CHECK(r[102] == 570 && f[102] == 1);
            CHECK(r[11] == 0 && f[11] == 2);
            CHECK(r[200] == 0 && f[200] == 2);  // angle missing from the answer
        }
    }

    // Recording a simulated drive: forward 0.5 m, turn 90 degrees, forward 0.3 m
    sim::World world;
    std::string err;
    if (!world.load(map_path, err)) {
        std::cerr << "Cannot load map " << map_path << ": " << err << "\n";
        return 1;
    }
    sim::SimRobot robot(world, world.start().value_or(geo::Pose2D{}));
    datalog::LogHeader header;
    header.source = "sim:" + map_path;
    datalog::DriveLogWriter log;
    CHECK(log.open(out_path, header));

    struct Move { double l, r, v; };
    const double quarter = 0.5 * robot.params().wheelbase_m * geo::kPi / 2.0 * 1000.0;
    const std::vector<Move> moves = {{500, 500, 150}, {-quarter, quarter, 100}, {300, 300, 150}};

    double t = 0.0;
    int last_scan = robot.scanCount();
    for (const auto& m : moves) {
        robot.driveWheels(m.l, m.r, m.v);
        log.command(t, m.l, m.r, m.v);
        while (robot.isMoving() && t < 60.0) {
            robot.step(0.02);
            t += 0.02;
            if (auto w = robot.getWheelPositions()) log.odom(t, *w);
            if (robot.scanCount() != last_scan) {
                last_scan = robot.scanCount();
                if (auto s = robot.getScan()) log.scan(t, *s);
            }
        }
    }
    log.event(t, "done");

    // Read back and check structure
    std::ifstream in(out_path);
    std::map<std::string, int> counts;
    std::string line;
    double last_t = -1.0;
    int lines = 0, bad = 0;
    bool time_ok = true;
    while (std::getline(in, line)) {
        ++lines;
        if (line.empty() || line.front() != '{' || line.back() != '}') ++bad;
        const std::string type = typeOf(line);
        ++counts[type];
        if (lines == 1) CHECK(type == "header");
        const auto tp = line.find("\"t\":");
        if (tp != std::string::npos) {
            const double tt = std::stod(line.substr(tp + 4));
            if (tt < last_t) time_ok = false;
            last_t = tt;
        }
        if (type == "scan") CHECK(intArray(line, "r").size() == 360);
    }
    CHECK(bad == 0);
    CHECK(time_ok);
    CHECK(lines == log.lines());
    CHECK(counts["header"] == 1);
    CHECK(counts["cmd"] == 3);
    CHECK(counts["event"] == 1);
    CHECK(counts["scan"] >= 10);   // ~9 s of driving at 5 Hz
    CHECK(counts["odom"] >= 300);

    std::cout << "drive log: " << lines << " lines, " << counts["scan"] << " scans, " << counts["odom"]
              << " odom -> " << out_path << "\n";
    if (failures == 0) std::cout << "All drive log tests passed\n";
    return failures == 0 ? 0 : 1;
}
