#pragma once
#include "Pose.hpp"
#include "BezierCurve.hpp"
#include "DriveLocalizerConstants.hpp"
#include "DriveEncoderLocalizer.hpp"
#include "PIDFController.hpp"
#include "Drivetrain.hpp"
#include "CurveChain.hpp"

class Follower {
public:
    std::unique_ptr<DriveEncoderLocalizer> localizer;
    std::unique_ptr<Drivetrain> drivetrain;
    BezierCurve currentPath;
    CurveChain currentPathChain;

    Follower( DriveLocalizerConstants constants,
                        PIDFCoefficients coeff,
                        Pose startPose = Pose()); 

    void followCurve(BezierCurve curve);
    void followCurveChain(CurveChain chain);
    void breakFollowing();
    void breakFollowingChain();
    void update();

    bool isBusy;

private:

    void followNextCurve();
    void setClosestTValue();
    void setTargetPose();
    void setHeadingError();
    double getForwardPower(double headingError);
    double getCorrectedHeading(double targetHeading);

    DriveLocalizerConstants constants;
    PIDFController headingPID;

    Pose currentPose;
    Pose targetPose;

    bool endTFlag = false;
    bool followingChainFlag = false;

    int currentChainIndex = -1;

    double lastClosestTValue;
    double closestTValue;

};