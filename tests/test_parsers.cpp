// Unit tests using REAL output recorded from Jawad's Neato D3 (GetCharger, 8 Oct 2026).
#include <cmath>
#include <iostream>

#include "core/geometry.hpp"
#include "core/odometry.hpp"
#include "neato/neato_protocol.hpp"

static int failures = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::cerr << "FAIL line " << __LINE__ << ": " #cond << "\n"; \
            ++failures;                                                   \
        }                                                                 \
    } while (0)

int main() {
    const std::string charger =
        "Label,Value\n"
        "FuelPercent,0\nBatteryOverTemp,0\nChargingActive,1\nChargingEnabled,1\n"
        "ConfidentOnFuel,0\nOnReservedFuel,0\nEmptyFuel,1\nBatteryFailure,0\n"
        "ExtPwrPresent,1\nThermistorPresent,1\nBattTempCAvg,22\nVBattV,11.36\n"
        "VExtV,18.80\nCharger_mAH,0\nDischarge_mAH,0\n";
    auto b = neato::parseBattery(charger);
    CHECK(b.has_value());
    CHECK(std::fabs(b->voltage_v - 11.36) < 1e-9);
    CHECK(std::fabs(b->external_v - 18.80) < 1e-9);
    CHECK(b->charging);
    CHECK(b->external_power);
    CHECK(b->fuel_percent == 0);

    const auto kv = neato::parseKeyValue("BrushSpeed,1200,,\nModel,BotVacD3Connected,905-0321,\n");
    CHECK(kv.at("BrushSpeed") == "1200");
    CHECK(kv.at("Model") == "BotVacD3Connected,905-0321");

    const std::string motors = "Parameter,Value\nLeftWheel_PositionInMM,1520\nRightWheel_PositionInMM,1498\n";
    auto w = neato::parseWheelPositions(motors);
    CHECK(w && w->left_mm == 1520 && w->right_mm == 1498);

    const std::string scan =
        "AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX\n"
        "0,1234,567,0\n1,0,0,8035\n2,2500,300,0\nROTATION_SPEED,5.12\n";
    auto s = neato::parseLdsScan(scan);
    CHECK(s && s->readings.size() == 3);
    CHECK(s->readings[0].valid && std::fabs(s->readings[0].range_m - 1.234) < 1e-9);
    CHECK(!s->readings[1].valid);
    CHECK(std::fabs(s->rotation_hz - 5.12) < 1e-9);

    // Lines recorded from the real D3 on 15 Oct 2026: flagged weak return and error 8035
    auto s2 = neato::parseLdsScan("AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX\n"
                                  "102,16954,46,0\n103,569,100,0\n11,0,0,8035\nROTATION_SPEED,5.02\n");
    CHECK(s2 && s2->readings.size() == 3);
    CHECK(s2->readings[0].weak && !s2->readings[0].valid && std::fabs(s2->readings[0].range_m - 0.570) < 1e-9);
    CHECK(s2->readings[1].valid && !s2->readings[1].weak);
    CHECK(!s2->readings[2].valid);

    const std::string digital =
        "Digital Sensor Name, Value\nSNSR_DC_JACK_CONNECT,0\nSNSR_DUSTBIN_IS_IN,1\n"
        "LSIDEBIT,0\nLFRONTBIT,1\nRSIDEBIT,0\nRFRONTBIT,0\n";
    auto bp = neato::parseBumpers(digital);
    CHECK(bp && bp->left_front && !bp->right_front && bp->any());

    // Odometry: 1 m straight, then a 90 degree turn on the spot
    core::Odometry odo(0.248);
    odo.update({0, 0});
    odo.update({1000, 1000});
    CHECK(std::fabs(odo.pose().x - 1.0) < 1e-9 && std::fabs(odo.pose().y) < 1e-9);
    const double arc = geo::kPi / 2 * 0.124 * 1000.0;
    odo.update({1000 - arc, 1000 + arc});
    CHECK(std::fabs(odo.pose().theta - geo::kPi / 2) < 1e-9);

    std::cout << (failures ? "Some tests FAILED\n" : "All parser and odometry tests passed\n");
    return failures ? 1 : 0;
}
