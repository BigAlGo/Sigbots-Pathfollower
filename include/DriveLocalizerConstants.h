#pragma once
#include <vector>
#include <cstdint>

struct DriveLocalizerConstants {
    std::vector<std::int8_t> leftMotorPorts;
    std::vector<std::int8_t> rightMotorPorts;  // negative port = reversed, same as any PROS motor
    double wheelDiameterInches;
    double externalGearRatio = 1.0;  // >1 if the wheel spins slower than the motor
    double trackWidthInches;         // distance between left and right wheel contact patches
};