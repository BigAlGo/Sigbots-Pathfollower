#include "Drivetrain.hpp"

Drivetrain::Drivetrain(std::vector<std::int8_t> leftPorts, std::vector<std::int8_t> rightPorts)
    : leftMotors(std::make_unique<pros::MotorGroup>(leftPorts)), rightMotors(std::make_unique<pros::MotorGroup>(rightPorts)) {
        
}

void Drivetrain::setLeftPower(double power) {
    double target = std::clamp(power * 127, -127.0, 127.0);
    currentLeftPower = target;
    leftMotors->move(target);
}

void Drivetrain::setRightPower(double power) {
    double target = std::clamp(power * 127, -127.0, 127.0);
    currentRightPower = target;
    rightMotors->move(target);
}

void Drivetrain::setLeftPowerSlew(double power) {
    double target = std::clamp(power * 127.0, -127.0, 127.0);
    double error = target - currentLeftPower;
    if (std::abs(error) > slewRate) {
        currentLeftPower += std::copysign(slewRate, error);
    } else {
        currentLeftPower = target;
    }
    leftMotors->move(currentLeftPower);
}

void Drivetrain::setRightPowerSlew(double power) {
    double target = std::clamp(power * 127.0, -127.0, 127.0);
    double error = target - currentRightPower;
    if (std::abs(error) > slewRate) {
        currentRightPower += std::copysign(slewRate, error);
    } else {
        currentRightPower = target;
    }
    rightMotors->move(currentRightPower);
}

void Drivetrain::stop() {
    currentLeftPower = 0;
    currentRightPower = 0;
    leftMotors->move(0);
    rightMotors->move(0);
}
