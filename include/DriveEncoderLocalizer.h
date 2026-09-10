#pragma once
#include "api.h"
#include "Pose.h"
#include "DriveLocalizerConstants.h"


using Matrix = std::vector<std::vector<double>>;

class DriveEncoderLocalizer {
public:
    DriveEncoderLocalizer(DriveLocalizerConstants constants,
                          Pose startPose = Pose());

    Pose getPose() const;
    void setStartPose(const Pose& pose);
    void setPose(const Pose& pose);
    void update();
    double getTotalHeading() const { return totalHeading; }

private:
    std::unique_ptr<pros::MotorGroup> leftMotors;
    std::unique_ptr<pros::MotorGroup> rightMotors;

    double inchesPerDegree;
    double trackWidth;

    Pose startPose;
    Pose displacementPose;
    Matrix prevRotationMatrix;
    double totalHeading = 0.0;

    double prevLeftDeg = 0.0;
    double prevRightDeg = 0.0;
    std::uint64_t prevTimeMicros = 0;

    void resetEncoders();
    double averagePosition(const pros::MotorGroup& group) const;
    Matrix getRobotDeltas();
};