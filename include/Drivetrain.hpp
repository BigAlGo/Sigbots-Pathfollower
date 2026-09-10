#pragma once
#include "api.h"

class Drivetrain {
private:
    std::unique_ptr<pros::MotorGroup> leftMotors;
    std::unique_ptr<pros::MotorGroup> rightMotors;

    double currentLeftPower = 0.0;
    double currentRightPower = 0.0;

    double slewRate = 20.0; // lower number is less responsive

public:
    Drivetrain(std::vector<std::int8_t> leftPorts, std::vector<std::int8_t> rightPorts);

    void setLeftPower(double power);
    void setRightPower(double power);

    void setLeftPowerSlew(double power);
    void setRightPowerSlew(double power);

    void stop();
};
