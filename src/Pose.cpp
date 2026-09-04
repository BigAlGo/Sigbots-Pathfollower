#include "Pose.h"
#include <cmath>

Pose::Pose() {
    x = 0.0;
    y = 0.0;
    heading = 0.0;
}
Pose::Pose(double X, double Y) {
    x = X;
    y = Y;
    heading = 0.0;
}
Pose::Pose(double X, double Y, double Heading) {
    x = X;
    y = Y;
    heading = Heading;
}

double Pose::distFrom(const Pose& otherPose) const {
    return sqrt( pow(otherPose.x - this->x, 2) + pow(otherPose.y - this->y, 2) );
}
