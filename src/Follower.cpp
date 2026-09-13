#include "Follower.hpp"

Follower::Follower( DriveLocalizerConstants constants,
                    PIDFCoefficients coeff,
                    Pose startPose) 
    :   drivetrain(std::make_unique<Drivetrain>(constants.leftMotorPorts, constants.rightMotorPorts)),
        localizer(std::make_unique<DriveEncoderLocalizer>(constants, startPose)),
        aTagManager(std::make_unique<AprilTagManager>(constants)),
        headingPID(coeff)
{
    this->constants = constants;
    this->currentPose = startPose;
    breakFollowing();
}

void Follower::update(bool updateWithCam) {
    if (updateWithCam) {
        // localizer->setPose(aTagManager->getRobotPose(currentPose));
    }
    double thisTickTurnPower;
    double thisTickDrivePower;
    localizer->update();
    currentPose = localizer->getPose();
    
    if (isBusy) {
        setClosestTValue();
        if (!followingChainFlag) {
            if (closestTValue < constants.endTValue) {
                setTargetPose();
                setHeadingError();

                double correctedHeading = getCorrectedHeading(headingPID.getTargetPosition());
                headingPID.updatePosition(correctedHeading);
                thisTickTurnPower = headingPID.run();
                thisTickDrivePower = getForwardPower(headingPID.getError());

                drivetrain->setLeftPowerSlew(thisTickDrivePower - thisTickTurnPower);
                drivetrain->setRightPowerSlew(thisTickDrivePower + thisTickTurnPower);
            } else {
                breakFollowingChain();
                drivetrain->stop();
            }
        
        } else {
            if (closestTValue < constants.endTValueChain) {
                setTargetPose();
                setHeadingError();

                double correctedHeading = getCorrectedHeading(headingPID.getTargetPosition());
                headingPID.updatePosition(correctedHeading);
                thisTickTurnPower = headingPID.run();

                thisTickDrivePower = getForwardPower(headingPID.getError());
                drivetrain->setLeftPowerSlew(thisTickDrivePower - thisTickTurnPower);
                drivetrain->setRightPowerSlew(thisTickDrivePower + thisTickTurnPower);
            } else {
                if ((currentChainIndex + 1) <= (currentPathChain.getLength() - 1)) {
                    followNextCurve();
                    // update(); carefull about velocity because delta t will be near 0
                } else {
                    breakFollowingChain();
                    drivetrain->stop();
                }
            }
        }
    }
}

void Follower::followCurve(BezierCurve curve) {
    breakFollowingChain();
    currentPath = curve;
    isBusy = true;
}

void Follower::followCurveChain(CurveChain chain) {
    breakFollowingChain();
    currentPathChain = chain;
    followNextCurve();
}

// private
void Follower::followNextCurve() {
    breakFollowing();
    followingChainFlag = true;
    isBusy = true;
    ++currentChainIndex;

    currentPath = currentPathChain.getCurve(currentChainIndex); 
}

void Follower::setClosestTValue() {
    if (!lastClosestTValue) { 
        lastClosestTValue = 0.0;
    }
    
    double minDist = 1000.0; 
    double resolution = static_cast<double>(constants.curveSearchResolutuon);
    
    for (int i = 0; ; ++i) {
        double tOffset = (i / 2) / resolution;
        
        if (tOffset * currentPath.length() >= constants.maxDistJump) {
            break; 
        }

        double searchTValue = (i % 2 == 0) 
            ? lastClosestTValue - tOffset 
            : lastClosestTValue + tOffset;

        if (searchTValue < 0.0 || searchTValue > 1.0) {
            continue;
        }

        double thisDist = currentPath.getPoseAtT(searchTValue).distFrom(currentPose);

        if (thisDist < minDist) {
            minDist = thisDist; 
            closestTValue = searchTValue;
        }
    }
    
    lastClosestTValue = closestTValue; 
}

void Follower::setTargetPose() {
    double aheadPoseDist = currentPath.getDistanceFromT(closestTValue) + constants.lookAheadDist;
    if (aheadPoseDist >= currentPath.length()) {
        if (followingChainFlag && (currentChainIndex + 1) <= (currentPathChain.getLength() - 1)) {
            double aheadAmount = aheadPoseDist - currentPath.length();
            targetPose = currentPathChain.getCurve(currentChainIndex + 1).getPoseAtDistance(aheadAmount);
            return;
        } else {
            aheadPoseDist = currentPath.length();
            endTFlag = true;
        }
    }
    targetPose = currentPath.getPoseAtDistance(aheadPoseDist);
}

void Follower::setHeadingError() {
    double targetHeading = std::atan2( (targetPose.y - currentPose.y),  (targetPose.x - currentPose.x) );
    headingPID.setTargetPosition(targetHeading);
}

double Follower::getForwardPower(double headingError) {
    double absError = std::abs(headingError);

    if (absError <= constants.maxPowerTreshold) {
        return constants.maxPower;

    } else if (absError >= std::numbers::pi - constants.maxPowerTreshold){
        return -constants.maxPower;

    } else {
        double numerator   = (std::numbers::pi / 2) - std::abs(headingError);
        double denominator = (std::numbers::pi / 2) - constants.maxPowerTreshold;

        double slope = numerator / denominator;
        return constants.maxPower * slope;
    }
}

// normalize
double Follower::getCorrectedHeading(double targetHeading) {
    double diff = currentPose.heading - targetHeading;
    double newHeading = currentPose.heading;
    while (diff > std::numbers::pi) {
        newHeading -= 2 * std::numbers::pi;
        diff -= 2 * std::numbers::pi;
    }
    while (diff <= -std::numbers::pi) {
        newHeading += 2 * std::numbers::pi;
        diff += 2 * std::numbers::pi;
    }

    return newHeading;
}

void Follower::breakFollowing() {
    isBusy = false;
    endTFlag = false;
    lastClosestTValue = 0.0;
    followingChainFlag = false;
    currentPath = BezierCurve();
}

void Follower::breakFollowingChain() {
    isBusy = false;
    endTFlag = false;
    currentChainIndex = -1;
    lastClosestTValue = 0.0;
    followingChainFlag = false;
    currentPath = BezierCurve();
    headingPID.reset();
}