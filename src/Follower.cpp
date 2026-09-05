#include "Follower.h"

Follower::Follower() {
    this->constants();
    this->currentPose = Pose();
}

Follower::Follower(const DriveLocalizerConstants& constants, const Pose& startPose = Pose()) {
    this->constants = constants;
    this->currentPose = startPose;
    // this->localizer = DriveEncoderLocalizer(constants, startPose);
}

void Follower::update() {
    localizer.update();
    currentPose = localizer.getPose();

    if (isBusy) {
        setClosestTValue();
        setTargetPose();
        setHeadingError();
        // pid heading
        // decide forward pow
        // decide to end
    }
}

void Follower::followCurve(const BezierCurve& curve) {
    currentPath = curve;
    isBusy = true;
}

void Follower::setClosestTValue() {
    if (!lastClosestTValue) {
        lastClosestTValue = 0;
    }
    
    double minDist = 1000.0;

    int i = 0;
    while ((i / 2) / curveSearchResolutuon > maxTJump) {
        double searchTValue;
        if (i % 2 == 0) {
            // Even
            searchTValue = lastClosestTValue - ((i/2) / curveSearchResolutuon);
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
    targetPose = currentPath.getPoseAtDistance(aheadPoseDist);
}

void Follower::setHeadingError() {

}