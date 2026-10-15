#include "neato/real_robot.hpp"

#include <cmath>
#include <sstream>

namespace neato {

bool RealRobot::connect(const std::string& port_name) { return port_.open(port_name); }

// Driving and the LiDAR need TestMode. With a healthy battery this is safe;
// with a deeply discharged battery the robot reboots (we saw this).
bool RealRobot::enableMotion(bool on) {
    if (on) {
        client_.command("TestMode On");
        client_.command("SetLDSRotation On");
    } else {
        client_.command("SetLDSRotation Off");
        client_.command("TestMode Off");
    }
    return true;
}

std::optional<LaserScan> RealRobot::getScan() { return parseLdsScan(client_.command("GetLDSScan")); }

std::optional<WheelPositions> RealRobot::getWheelPositions() {
    return parseWheelPositions(client_.command("GetMotors LeftWheel RightWheel"));
}

bool RealRobot::driveWheels(double left_mm, double right_mm, double speed_mm_s) {
    std::ostringstream cmd;
    cmd << "SetMotor LWheelDist " << std::lround(left_mm) << " RWheelDist " << std::lround(right_mm)
        << " Speed " << std::lround(speed_mm_s);
    client_.command(cmd.str());
    return true;
}

std::optional<BatteryStatus> RealRobot::getBattery() { return parseBattery(client_.command("GetCharger")); }

std::optional<Bumpers> RealRobot::getBumpers() { return parseBumpers(client_.command("GetDigitalSensors")); }

}  // namespace neato
