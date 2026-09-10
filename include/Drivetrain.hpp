#pragma once
#include "api.h"

class Drivetrain {
    
public:
    Drivetrain(std::vector<std::int8_t> leftPorts, std::vector<std::int8_t> rightPorts);
    
    void setLeftPower(double power);
    
    void setRightPower(double power);

    void stop();

private:
    std::unique_ptr<pros::MotorGroup> leftMotors;
    std::unique_ptr<pros::MotorGroup> rightMotors;
};