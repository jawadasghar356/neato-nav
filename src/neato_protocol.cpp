#include "neato/neato_protocol.hpp"

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <sstream>
#include <thread>

namespace neato {

namespace {

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

double toDouble(const std::map<std::string, std::string>& kv, const std::string& key, bool& ok) {
    auto it = kv.find(key);
    if (it == kv.end()) { ok = false; return 0.0; }
    return std::atof(it->second.c_str());
}

}  // namespace

std::string NeatoClient::command(const std::string& cmd, int timeout_ms) {
    // The robot sometimes misses the start of a command right after it has sent a long
    // answer (seen on the real D3: "Unknown Cmd: 'SetLDSRotation'"). Retry once.
    std::string answer = commandOnce(cmd, timeout_ms);
    if (answer.rfind("Unknown Cmd", 0) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        answer = commandOnce(cmd, timeout_ms);
    }
    return answer;
}

std::string NeatoClient::commandOnce(const std::string& cmd, int timeout_ms) {
    using clock = std::chrono::steady_clock;
    port_.flushInput();  // drop boot messages or old output
    if (!port_.write(cmd + "\n")) return "";

    std::string answer;
    char buf[512];
    const auto start = clock::now();
    auto last_data = start;
    while (true) {
        const int n = port_.readSome(buf, sizeof(buf), 50);
        const auto now = clock::now();
        if (n > 0) {
            answer.append(buf, static_cast<size_t>(n));
            last_data = now;
            if (answer.find('\x1A') != std::string::npos) break;  // normal end of answer
        }
        const auto since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        const auto idle = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_data).count();
        if (since_start > timeout_ms) break;
        if (!answer.empty() && idle > 300) break;  // fallback if no Ctrl-Z arrives
    }

    // Clean up: cut at Ctrl-Z, remove '\r', drop the echoed command on the first line
    const auto z = answer.find('\x1A');
    if (z != std::string::npos) answer.resize(z);
    std::string clean;
    for (char c : answer)
        if (c != '\r') clean.push_back(c);
    const auto nl = clean.find('\n');
    if (nl != std::string::npos && trim(clean.substr(0, nl)) == trim(cmd)) clean.erase(0, nl + 1);
    return clean;
}

std::map<std::string, std::string> parseKeyValue(const std::string& text) {
    std::map<std::string, std::string> kv;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        const auto comma = line.find(',');
        if (comma == std::string::npos) continue;
        std::string key = trim(line.substr(0, comma));
        std::string value = trim(line.substr(comma + 1));
        while (!value.empty() && value.back() == ',') value.pop_back();  // "BrushSpeed,1200,,"
        if (!key.empty()) kv[key] = value;
    }
    return kv;
}

std::optional<BatteryStatus> parseBattery(const std::string& text) {
    const auto kv = parseKeyValue(text);
    bool ok = true;
    BatteryStatus b;
    b.voltage_v = toDouble(kv, "VBattV", ok);
    b.external_v = toDouble(kv, "VExtV", ok);
    b.fuel_percent = static_cast<int>(toDouble(kv, "FuelPercent", ok));
    b.charging = toDouble(kv, "ChargingActive", ok) != 0.0;
    b.external_power = toDouble(kv, "ExtPwrPresent", ok) != 0.0;
    b.temperature_c = toDouble(kv, "BattTempCAvg", ok);
    if (!ok) return std::nullopt;
    return b;
}

std::optional<WheelPositions> parseWheelPositions(const std::string& text) {
    const auto kv = parseKeyValue(text);
    bool ok = true;
    WheelPositions w;
    w.left_mm = toDouble(kv, "LeftWheel_PositionInMM", ok);
    w.right_mm = toDouble(kv, "RightWheel_PositionInMM", ok);
    if (!ok) return std::nullopt;
    return w;
}

std::optional<Bumpers> parseBumpers(const std::string& text) {
    const auto kv = parseKeyValue(text);
    bool ok = true;
    Bumpers b;
    b.left_front = toDouble(kv, "LFRONTBIT", ok) != 0.0;
    b.left_side = toDouble(kv, "LSIDEBIT", ok) != 0.0;
    b.right_front = toDouble(kv, "RFRONTBIT", ok) != 0.0;
    b.right_side = toDouble(kv, "RSIDEBIT", ok) != 0.0;
    if (!ok) return std::nullopt;
    return b;
}

// Expected format (360 data lines):
//   AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX
//   0,1234,567,0
//   ...
//   ROTATION_SPEED,5.12
std::optional<LaserScan> parseLdsScan(const std::string& text) {
    LaserScan scan;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.rfind("ROTATION_SPEED", 0) == 0) {
            scan.rotation_hz = std::atof(line.substr(line.find(',') + 1).c_str());
            continue;
        }
        if (line.empty() || !(std::isdigit(static_cast<unsigned char>(line[0])))) continue;
        std::istringstream ls(line);
        std::string a, d, i, e;
        if (!std::getline(ls, a, ',') || !std::getline(ls, d, ',') ||
            !std::getline(ls, i, ',') || !std::getline(ls, e, ','))
            continue;
        LaserReading r;
        r.angle_deg = std::atoi(a.c_str());
        const int dist_mm = std::atoi(d.c_str());
        r.intensity = std::atoi(i.c_str());
        const long err = std::strtol(e.c_str(), nullptr, 16);
        // Real D3 data: some distances come back as 16384 + d (e.g. 16954 = 570 mm).
        // Bit 14 (0x4000) is the LDS "signal strength warning" flag, not part of the range.
        r.weak = (dist_mm & 0x4000) != 0;
        r.range_m = (dist_mm & 0x3FFF) / 1000.0;
        r.valid = (err == 0 && dist_mm > 0 && !r.weak);
        scan.readings.push_back(r);
    }
    if (scan.readings.empty()) return std::nullopt;
    return scan;
}

}  // namespace neato
