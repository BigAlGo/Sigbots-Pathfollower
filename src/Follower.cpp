#include "Follower.h"

Follower::Follower( std::initializer_list<std::int8_t> leftPorts,
                    std::initializer_list<std::int8_t> rightPorts,
                    const DriveLocalizerConstants& constants,
                    const Pose& startPose = Pose(), 
                    const PIDFCoefficients& coeff)
    :   drivetrain(leftPorts, rightPorts),
        localizer(constants, startPose),
        headingPID(coeff),
        currentPath({Pose(0,0), Pose(1,1), Pose (0,1)})
{

    this->constants = constants;
    this->currentPose = startPose;
}

void Follower::update() {
    double thisTickTurnPower;
    double thisTickDrivePower;
    localizer.update();
    currentPose = localizer.getPose();

    if (isBusy) {
        setClosestTValue();
        if (closestTValue < endTValue) {
            setTargetPose();
            setHeadingError();
            thisTickTurnPower = headingPID.run();
            thisTickDrivePower = getForwardPower(headingPID.getError());
            drivetrain.setLeftPower(thisTickDrivePower + thisTickTurnPower);
            drivetrain.setRightPower(thisTickDrivePower - thisTickTurnPower);
        } else {
            breakFollowing();
            drivetrain.stop();
        }
    }
}

void Follower::followCurve(const BezierCurve& curve) {
    breakFollowing();
    currentPath = curve;
    isBusy = true;
}

void Follower::setClosestTValue() {
    if (!lastClosestTValue) {
        lastClosestTValue = 0;
    }
    
    double minDist = 1000.0;

    int i = 0;
    // Only an estimation, doesnt factor in curves, I think
    while ((i / 2) / curveSearchResolutuon * currentPath.length() < maxDistJump) {
        double searchTValue;
        if (i % 2 == 0) {
            // Even
            searchTValue = lastClosestTValue - ((  i   / 2) / curveSearchResolutuon);
        } else {
            // Odd
            searchTValue = lastClosestTValue + (((i-1) / 2) / curveSearchResolutuon);
        }

        if (searchTValue < 0 or searchTValue > 1) {
            ++i;
            continue;
        }

        double thisDist = currentPath.getPoseAtT(searchTValue).distFrom(currentPose);

        if (thisDist < minDist) {
            thisDist == minDist;
            closestTValue = searchTValue;
        }
    }
}

void Follower::setTargetPose() {
    double aheadPoseDist = currentPath.getDistanceFromT(closestTValue) + lookAheadDist;
    if (aheadPoseDist >= currentPath.length()) {
        aheadPoseDist = currentPath.length();
        endTFlag = true;
    }
    targetPose = currentPath.getPoseAtDistance(aheadPoseDist);
}

void Follower::setHeadingError() {
    double targetHeading = std::atan2( (targetPose.y - currentPose.y),  (targetPose.x - currentPose.x) );
    headingPID.setTargetPosition(targetHeading);
}

double Follower::getForwardPower(double headingError) {
    double absError = std::abs(headingError);

    if (absError <= maxPowerTreshold) {
        return maxPower;

    } else if (absError >= std::numbers::pi - maxPowerTreshold){
        return -maxPower;

    } else {
        double numerator   = (std::numbers::pi / 2) - std::abs(headingError);
        double denominator = (std::numbers::pi / 2) - maxPowerTreshold;

        double slope = numerator / denominator;
        return maxPower * slope;
    }
}

void Follower::breakFollowing() {
    endTFlag = false;
    headingPID.reset();
    currentPath = BezierCurve();
}