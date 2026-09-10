#include "Pose.h"
#include "BezierCurve.h"
#include "DriveLocalizerConstants.h"
#include "DriveEncoderLocalizer.h"
#include "PIDFController.h"
#include "Drivetrain.h"

class Follower {
public:
    std::unique_ptr<DriveEncoderLocalizer> localizer;
    std::unique_ptr<Drivetrain> drivetrain;
    BezierCurve currentPath;

    Follower( DriveLocalizerConstants constants,
                        PIDFCoefficients coeff,
                        Pose startPose = Pose()); 

    void followCurve(BezierCurve curve);
    void breakFollowing();
    void update();

    bool isBusy;

private:

    void setClosestTValue();
    void setTargetPose();
    void setHeadingError();
    double getForwardPower(double headingError);

    DriveLocalizerConstants constants;
    PIDFController headingPID;

    Pose currentPose;
    Pose targetPose;

    bool endTFlag = false;

    // Should be put in localizer constants eventually
    int curveSearchResolutuon = 300;
    double lastClosestTValue;
    double closestTValue;
    double endTValue = 0.95;
    double maxDistJump = 20;
    double lookAheadDist = 7;
    double maxPower = 0.9;
    double maxPowerTreshold = std::numbers::pi / 16;

};