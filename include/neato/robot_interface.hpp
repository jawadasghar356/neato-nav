// The one interface every robot backend implements: the real Neato (over USB)
// and the simulator. All algorithms are written against IRobot, so they run
// unchanged on both. This is the "hardware abstraction layer".
#pragma once
#include <optional>
#include <vector>

namespace neato {

struct LaserReading {
    int angle_deg{0};     // 0..359, 0 = straight ahead, counter-clockwise (verify on hardware!)
    double range_m{0.0};  // distance in metres
    int intensity{0};     // signal strength reported by the LDS
    bool valid{false};    // false if the LDS reported an error or no return
    bool weak{false};     // LDS set its "signal strength warning" flag (0x4000 in DistInMM)
};

struct LaserScan {
    std::vector<LaserReading> readings;  // normally 360 entries, one per degree
    double rotation_hz{0.0};             // LDS turret speed (about 5 Hz)
};

// Cumulative wheel travel measured by the encoders since power-on
struct WheelPositions {
    double left_mm{0.0};
    double right_mm{0.0};
};

struct BatteryStatus {
    double voltage_v{0.0};
    double external_v{0.0};
    int fuel_percent{0};
    bool charging{false};
    bool external_power{false};
    double temperature_c{0.0};
};

// The four bumper switches (GetDigitalSensors: LFRONTBIT, LSIDEBIT, RFRONTBIT, RSIDEBIT)
struct Bumpers {
    bool left_front{false};
    bool left_side{false};
    bool right_front{false};
    bool right_side{false};
    bool any() const { return left_front || left_side || right_front || right_side; }
};

class IRobot {
public:
    virtual ~IRobot() = default;

    // Real robot: TestMode on/off and LDS spinning. Simulator: no-op.
    virtual bool enableMotion(bool on) = 0;

    virtual std::optional<LaserScan> getScan() = 0;
    virtual std::optional<WheelPositions> getWheelPositions() = 0;

    // Move each wheel by the given distance (mm); speed applies to the faster wheel.
    // This mirrors the Neato command "SetMotor LWheelDist .. RWheelDist .. Speed ..".
    virtual bool driveWheels(double left_mm, double right_mm, double speed_mm_s) = 0;

    virtual std::optional<BatteryStatus> getBattery() = 0;

    virtual std::optional<Bumpers> getBumpers() = 0;
};

}  // namespace neato
