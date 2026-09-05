#include "Pose.h"
#include "BezierCurve.h"
#include "DriveLocalizerConstants.h"
#include "DriveEncoderLocalizer.h"

class Follower {
public:
    DriveEncoderLocalizer localizer;
    BezierCurve currentPath;

    Follower();
    Follower(const DriveLocalizerConstants& constants, const Pose& startPose = Pose());
    bool isBusy() const { return isBusy; }
    void followCurve(const BezierCurve& curve);
    void update();

private:

    void setClosestTValue();
    void setClosestTValue();
    void setTargetPose();
    void setTargetHeading();
    void setHeadingError();


    DriveLocalizerConstants constants;
    Pose currentPose;
    Pose targetPose;
    int curveSearchResolutuon = 300;
    double lastClosestTValue;
    double closestTValue;
    double endTValue = 0.95;
    double maxTJump = 0.2;
    double lookAheadDist = 7;
    bool isBusy;
};