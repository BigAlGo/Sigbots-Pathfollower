#pragma once
#include "api.h"

// #include <vector>
// #include <cstdint>

struct DriveLocalizerConstants {
    DriveLocalizerConstants() = default;

    std::vector<std::int8_t> leftMotorPorts = {1, 11};
    std::vector<std::int8_t> rightMotorPorts = {-20, -10};

    //Physical Constants
    double wheelDiameterInches = 3.54; // Almost tuned
    double externalGearRatio = 1.0;
    double trackWidthInches =  9.75; // distance between left and right wheel contact patches

    //Follower Constants
    int curveSearchResolutuon = 300;
    double endTValue = 0.95;
    double endTValueChain = 0.95;
    double maxDistJump = 20;
    double lookAheadDist = 12;
    double maxPower = 0.25;
    double maxPowerTreshold = std::numbers::pi / 16;
    double forwardSlipFactor = 0.908;
    double turnSlipFactor = 0.87;

};