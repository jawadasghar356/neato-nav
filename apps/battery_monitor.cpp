// Polls GetCharger on the real robot at a fixed interval, prints the values and appends them
// to a CSV file. Works on dock power without TestMode. The serial port must not be open in
// another program (e.g. PuTTY).
//
// Usage: battery_monitor [port] [interval_s] [csv]
//        defaults: /dev/ttyACM0 (Windows: COM9), 60, battery_log.csv
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "neato/neato_protocol.hpp"
#include "neato/real_robot.hpp"

static std::string timestamp() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream s;
    s << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return s.str();
}

int main(int argc, char** argv) {
    #ifdef _WIN32
    const std::string port = argc > 1 ? argv[1] : "COM9";
#else
    const std::string port = argc > 1 ? argv[1] : "/dev/ttyACM0";
#endif
    const int interval_s = argc > 2 ? std::stoi(argv[2]) : 60;
    const std::string csv_path = argc > 3 ? argv[3] : "battery_log.csv";

    neato::RealRobot robot;
    if (!robot.connect(port)) {
        std::cerr << "Error: " << robot.lastError() << "\n";
        return 1;
    }

    // Identify the robot once
    const auto version = neato::parseKeyValue(robot.raw("GetVersion"));
    auto model = version.find("Model");
    auto sw = version.find("Software");
    std::cout << "Connected to " << port << ": "
              << (model != version.end() ? model->second : std::string("unknown model"))
              << ", software " << (sw != version.end() ? sw->second : std::string("?")) << "\n";

    std::ofstream csv(csv_path, std::ios::app);
    csv << "time,voltage_v,external_v,charging,fuel_percent,temperature_c\n";
    std::cout << "Logging every " << interval_s << " s to " << csv_path << " (Ctrl+C to stop)\n\n";
    std::cout << std::fixed << std::setprecision(2);

    while (true) {
        const auto b = robot.getBattery();
        const std::string now = timestamp();
        if (b) {
            std::cout << now << "  battery " << b->voltage_v << " V  dock " << b->external_v << " V  "
                      << (b->charging ? "CHARGING" : "not charging") << "  fuel " << b->fuel_percent
                      << " %  temp " << b->temperature_c << " C\n";
            csv << now << ',' << b->voltage_v << ',' << b->external_v << ',' << b->charging << ','
                << b->fuel_percent << ',' << b->temperature_c << '\n';
            csv.flush();
            if (b->temperature_c > 45.0) std::cout << "WARNING: battery is warm, take the robot off the dock!\n";
        } else {
            std::cout << now << "  no valid answer (robot rebooting or cable loose?)\n";
        }
        std::this_thread::sleep_for(std::chrono::seconds(interval_s));
    }
}
