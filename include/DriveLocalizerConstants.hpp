#pragma once
#include "api.h"

// #include <vector>
// #include <cstdint>

struct DriveLocalizerConstants {
    std::vector<std::int8_t> leftMotorPorts = {1, 11};
    std::vector<std::int8_t> rightMotorPorts = {-20, -10};
    double wheelDiameterInches = 3.54; // Almost tuned
    double externalGearRatio = 1.0;
    double trackWidthInches =  9.75; // distance between left and right wheel contact patches

    DriveLocalizerConstants(){};
};