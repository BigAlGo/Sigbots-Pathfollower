#include "Drivetrain.h"

Drivetrain::Drivetrain(std::initializer_list<std::int8_t> leftPorts, std::initializer_list<std::int8_t> rightPorts)
    : leftMotors(leftPorts), rightMotors(rightPorts) {
        // todo configure forward back
}

void Drivetrain::setLeftPower(double power) {
    leftMotors.move(power);
}

void Drivetrain::setRightPower(double power) {
    rightMotors.move(power);
}

void Drivetrain::stop() {
    leftMotors.move(0);
    rightMotors.move(0);
}