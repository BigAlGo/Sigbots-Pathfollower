#include "Drivetrain.hpp"

Drivetrain::Drivetrain(std::vector<std::int8_t> leftPorts, std::vector<std::int8_t> rightPorts)
    : leftMotors(std::make_unique<pros::MotorGroup>(leftPorts)), rightMotors(std::make_unique<pros::MotorGroup>(rightPorts)) {
        // todo configure forward back
}

void Drivetrain::setLeftPower(double power) {
    leftMotors->move(std::clamp(power * 127, -127.0, 127.0));
}

void Drivetrain::setRightPower(double power) {
    rightMotors->move(std::clamp(power * 127, -127.0, 127.0));
}

void Drivetrain::stop() {
    leftMotors->move(0);
    rightMotors->move(0);
}