#pragma once
#include "api.h"

// #include <vector>
// #include <cstdint>

struct DriveLocalizerConstants {
    std::vector<std::int8_t> leftMotorPorts = {1, 11};
    std::vector<std::int8_t> rightMotorPorts = {-20, -10};  // negative port = reversed, same as any PROS motor
    double wheelDiameterInches = 2.75;
    double externalGearRatio = 1.0;
    double trackWidthInches =  9.75; // distance between left and right wheel contact patches

    DriveLocalizerConstants(){};
};