#include "Pose.h"
#include "BezierCurve.h"
#include "DriveLocalizerConstants.h"
#include "DriveEncoderLocalizer.h"
#include "PIDFController.h"
#include "Drivetrain.h"

class Follower {
public:
    DriveEncoderLocalizer localizer;
    BezierCurve currentPath;
    Drivetrain drivetrain;

    Follower::Follower( std::initializer_list<std::int8_t> leftPorts,
                        std::initializer_list<std::int8_t> rightPorts,
                        const DriveLocalizerConstants& constants,
                        const Pose& startPose = Pose(), 
                        const PIDFCoefficients& coeff);
    bool isBusy() const { return isBusy; }
    void followCurve(const BezierCurve& curve);
    void breakFollowing();
    void update();

private:

    void setClosestTValue();
    void setClosestTValue();
    void setTargetPose();
    void setHeadingError();
    double getForwardPower(double headingError);

    DriveLocalizerConstants constants;
    PIDFController headingPID;
    Pose currentPose;
    Pose targetPose;

    bool endTFlag = false;
    bool isBusy;


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