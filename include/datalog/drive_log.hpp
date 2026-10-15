// Recording of a drive as JSON Lines: one JSON object per line, one line per measurement.
//
// Line types (all times "t" in seconds since the start of the recording, monotonic clock):
//   header  {"type":"header","source":..,"start_utc":..,"wheelbase_m":..,"lds_offset_m":..,
//            "angle_convention":..}
//   scan    {"t":..,"type":"scan","hz":5.02,"r":[360 x mm],"i":[360 x intensity],"f":[360 x flag]}
//           flag 0 = valid, 1 = weak (LDS strength warning, range kept), 2 = no reading (range 0)
//           array index = angle in degrees
//   odom    {"t":..,"type":"odom","l":mm,"r":mm}          cumulative wheel travel
//   cmd     {"t":..,"type":"cmd","l":mm,"r":mm,"v":mm_s}   SetMotor command sent to the robot
//   event   {"t":..,"type":"event","msg":"..."}
//
// JSON Lines keeps the file appendable while recording and trivial to read from Python.
#pragma once
#include <fstream>
#include <string>

#include "neato/robot_interface.hpp"

namespace datalog {

struct LogHeader {
    std::string source;            // e.g. "real:/dev/ttyACM0" or "sim:maps/apartment.map"
    double wheelbase_m = 0.248;
    double lds_offset_m = -0.09;   // LDS position along the robot x axis, relative to the axle
    std::string angle_convention = "deg, 0 = forward, counter-clockwise (unverified on hardware)";
};

class DriveLogWriter {
public:
    bool open(const std::string& path, const LogHeader& header);
    bool isOpen() const { return out_.is_open(); }
    long lines() const { return lines_; }

    void scan(double t, const neato::LaserScan& scan);
    void odom(double t, const neato::WheelPositions& wheels);
    void command(double t, double left_mm, double right_mm, double speed_mm_s);
    void event(double t, const std::string& message);

private:
    void finishLine();
    std::ofstream out_;
    long lines_ = 0;
};

// Escapes a string for use inside JSON quotes
std::string jsonEscape(const std::string& s);

}  // namespace datalog
