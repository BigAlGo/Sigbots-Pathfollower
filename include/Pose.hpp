#pragma once
#include "api.h"

class Pose {
public:
    double x;
    double y;
    double heading;

    Pose();
    Pose(double X, double Y);
    Pose(double X, double Y, double Heading);

    double distFrom(const Pose& otherPose) const;

    Pose operator+(const Pose& other) const;
    Pose operator-(const Pose& other) const;
};
