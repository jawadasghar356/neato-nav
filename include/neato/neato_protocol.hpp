// Text protocol of the Neato Botvac (D3-D7) over its USB serial port.
// The robot answers each command with lines of text and ends the answer with
// Ctrl-Z (0x1A). The parsers are free functions so they can be unit-tested
// with recorded robot output, without hardware.
#pragma once
#include <map>
#include <optional>
#include <string>

#include "neato/robot_interface.hpp"
#include "neato/serial_port.hpp"

namespace neato {

class NeatoClient {
public:
    explicit NeatoClient(SerialPort& port) : port_(port) {}

    // Send one command (e.g. "GetCharger") and return the robot's answer as text,
    // without the echoed command line. Empty string on timeout.
    std::string command(const std::string& cmd, int timeout_ms = 2000);

private:
    std::string commandOnce(const std::string& cmd, int timeout_ms);
    SerialPort& port_;
};

// "Label,Value" style answers (GetCharger, GetMotors, GetVersion, ...)
std::map<std::string, std::string> parseKeyValue(const std::string& text);

std::optional<BatteryStatus> parseBattery(const std::string& getChargerText);
std::optional<WheelPositions> parseWheelPositions(const std::string& getMotorsText);
std::optional<LaserScan> parseLdsScan(const std::string& getLdsScanText);
std::optional<Bumpers> parseBumpers(const std::string& getDigitalSensorsText);

}  // namespace neato
