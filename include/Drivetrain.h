#pragma once
#include "api.h"

class Drivetrain {
    
public:
    Drivetrain(std::initializer_list<std::int8_t> leftPorts, std::initializer_list<std::int8_t> rightPorts);
    
    void setLeftPower(double power);
    
    void setRightPower(double power);

    void stop();
    
private:
    pros::MotorGroup leftMotors;
    pros::MotorGroup rightMotors;
};