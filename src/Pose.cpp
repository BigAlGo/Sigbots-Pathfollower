#include "Pose.hpp"

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

double Pose::distFrom(const Pose& other) const {
    return sqrt( pow(other.x - this->x, 2) + pow(other.y - this->y, 2) );
}

Pose Pose::operator+(const Pose& other) const {
    return Pose(x + other.x, y + other.y, heading + other.heading);
}
Pose Pose::operator-(const Pose& other) const {
    return Pose(x - other.x, y - other.y, heading - other.heading);
}
