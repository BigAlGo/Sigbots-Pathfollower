#pragma once
#include "api.h"

// #include <vector>
// #include <cstdint>

struct DriveLocalizerConstants {
    std::vector<std::int8_t> leftMotorPorts;
    std::vector<std::int8_t> rightMotorPorts;  // negative port = reversed, same as any PROS motor
    double wheelDiameterInches;
    double externalGearRatio = 1.0;
    double trackWidthInches;         // distance between left and right wheel contact patches

    DriveLocalizerConstants();

};