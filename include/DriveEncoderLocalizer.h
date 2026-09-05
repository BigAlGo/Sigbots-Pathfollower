#pragma once
#include <vector>
#include <cstdint>
#include "pros/motors.hpp"
#include "Pose.h"
#include "main.h"
#include "DriveLocalizerConstants.h"


using Matrix = std::vector<std::vector<double>>;

class DriveEncoderLocalizer {
public:
    DriveEncoderLocalizer(const DriveLocalizerConstants& constants,
                          const Pose& startPose = Pose());

    Pose getPose() const;
    void setStartPose(const Pose& pose);
    void setPose(const Pose& pose);
    void update();
    double getTotalHeading() const { return totalHeading_; }

private:
    pros::MotorGroup leftMotors_;
    pros::MotorGroup rightMotors_;

    double inchesPerDegree_;
    double trackWidth_;

    Pose startPose_;
    Pose displacementPose_;
    Matrix prevRotationMatrix_;
    double totalHeading_ = 0.0;

    double prevLeftDeg_ = 0.0;
    double prevRightDeg_ = 0.0;
    std::uint64_t prevTimeMicros_ = 0;

    void resetEncoders();
    double averagePosition(pros::MotorGroup& group) const;
    Matrix getRobotDeltas();
};