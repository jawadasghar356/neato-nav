// IRobot backend for the physical Neato, connected by USB (or later via the ESP32).
#pragma once
#include <string>

#include "neato/neato_protocol.hpp"
#include "neato/robot_interface.hpp"
#include "neato/serial_port.hpp"

namespace neato {

class RealRobot : public IRobot {
public:
    bool connect(const std::string& port_name);  // e.g. "COM9"
    const std::string& lastError() const { return port_.lastError(); }

    // Raw access for experiments, e.g. robot.raw("GetVersion")
    std::string raw(const std::string& cmd) { return client_.command(cmd); }

    bool enableMotion(bool on) override;
    std::optional<LaserScan> getScan() override;
    std::optional<WheelPositions> getWheelPositions() override;
    bool driveWheels(double left_mm, double right_mm, double speed_mm_s) override;
    std::optional<BatteryStatus> getBattery() override;
    std::optional<Bumpers> getBumpers() override;

private:
    SerialPort port_;
    NeatoClient client_{port_};
};

}  // namespace neato
