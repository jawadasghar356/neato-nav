#include "datalog/drive_log.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace datalog {

namespace {

std::string utcNow() {
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
#ifdef _WIN32
    gmtime_s(&tm, &now);
#else
    gmtime_r(&now, &tm);
#endif
    std::ostringstream s;
    s << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return s.str();
}

}  // namespace

std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) continue;  // drop other control chars
                out += c;
        }
    }
    return out;
}

bool DriveLogWriter::open(const std::string& path, const LogHeader& h) {
    out_.open(path, std::ios::out | std::ios::trunc);
    if (!out_) return false;
    out_ << std::fixed << std::setprecision(3);
    out_ << "{\"type\":\"header\",\"source\":\"" << jsonEscape(h.source) << "\",\"start_utc\":\"" << utcNow()
         << "\",\"wheelbase_m\":" << h.wheelbase_m << ",\"lds_offset_m\":" << h.lds_offset_m
         << ",\"angle_convention\":\"" << jsonEscape(h.angle_convention) << "\"}";
    finishLine();
    return true;
}

void DriveLogWriter::scan(double t, const neato::LaserScan& scan) {
    if (!out_) return;
    // Arrays indexed by angle, so a missing line in the robot's answer cannot shift the data
    std::array<int, 360> r{}, in{}, f{};
    f.fill(2);
    for (const auto& b : scan.readings) {
        if (b.angle_deg < 0 || b.angle_deg >= 360) continue;
        const auto k = static_cast<size_t>(b.angle_deg);
        in[k] = b.intensity;
        if (b.valid) {
            r[k] = static_cast<int>(std::lround(b.range_m * 1000.0));
            f[k] = 0;
        } else if (b.weak) {
            r[k] = static_cast<int>(std::lround(b.range_m * 1000.0));
            f[k] = 1;
        }
    }
    auto writeArray = [this](const char* key, const std::array<int, 360>& a) {
        out_ << ",\"" << key << "\":[";
        for (size_t k = 0; k < a.size(); ++k) out_ << (k ? "," : "") << a[k];
        out_ << ']';
    };
    out_ << "{\"t\":" << t << ",\"type\":\"scan\",\"hz\":" << std::setprecision(2) << scan.rotation_hz
         << std::setprecision(3);
    writeArray("r", r);
    writeArray("i", in);
    writeArray("f", f);
    out_ << '}';
    finishLine();
}

void DriveLogWriter::odom(double t, const neato::WheelPositions& w) {
    if (!out_) return;
    out_ << "{\"t\":" << t << ",\"type\":\"odom\",\"l\":" << std::setprecision(1) << w.left_mm
         << ",\"r\":" << w.right_mm << std::setprecision(3) << '}';
    finishLine();
}

void DriveLogWriter::command(double t, double left_mm, double right_mm, double speed_mm_s) {
    if (!out_) return;
    out_ << "{\"t\":" << t << ",\"type\":\"cmd\",\"l\":" << std::lround(left_mm) << ",\"r\":"
         << std::lround(right_mm) << ",\"v\":" << std::lround(speed_mm_s) << '}';
    finishLine();
}

void DriveLogWriter::event(double t, const std::string& message) {
    if (!out_) return;
    out_ << "{\"t\":" << t << ",\"type\":\"event\",\"msg\":\"" << jsonEscape(message) << "\"}";
    finishLine();
}

void DriveLogWriter::finishLine() {
    out_ << '\n';
    out_.flush();  // a crash or Ctrl+C must not lose the recording
    ++lines_;
}

}  // namespace datalog
