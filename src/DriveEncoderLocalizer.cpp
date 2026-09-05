
#include "DriveEncoderLocalizer.h"


static Matrix rotationMatrix(double heading) {
    Matrix m(3, std::vector<double>(3, 0.0));   // 3x3, zero-filled
    m[0][0] = std::cos(heading);  m[0][1] = -std::sin(heading);
    m[1][0] = std::sin(heading);  m[1][1] = std::cos(heading);
    m[2][2] = 1.0;
    return m;
}

static Matrix multiply(const Matrix& a, const Matrix& b) {
    size_t rows = a.size(), inner = b.size(), cols = b[0].size();
    Matrix result(rows, std::vector<double>(cols, 0.0));
    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            for (size_t k = 0; k < inner; ++k)
                result[i][j] += a[i][k] * b[k][j];
    return result;
}


DriveEncoderLocalizer::DriveEncoderLocalizer(const DriveLocalizerConstants& c, const Pose& startPose)
    : leftMotors_(c.leftMotorPorts),
      rightMotors_(c.rightMotorPorts),
      trackWidth_(c.trackWidthInches)
{
    inchesPerDegree_ = (c.wheelDiameterInches * M_PI) / 360.0 / c.externalGearRatio;
    startPose_ = startPose;
    displacementPose_ = Pose();
    prevRotationMatrix_ = rotationMatrix(0.0);
    resetEncoders();
    prevTimeMicros_ = pros::micros();
}

double DriveEncoderLocalizer::averagePosition(pros::MotorGroup& group) const {
    std::vector<double> positions = group.get_position_all();
    double sum = 0.0;
    for (double p : positions) sum += p;
    return sum / positions.size();
}

void DriveEncoderLocalizer::resetEncoders() {
    leftMotors_.tare_position_all();
    rightMotors_.tare_position_all();
    prevLeftDeg_ = 0.0;
    prevRightDeg_ = 0.0;
}

Matrix DriveEncoderLocalizer::getRobotDeltas() {
    double leftDeg = averagePosition(leftMotors_);
    double rightDeg = averagePosition(rightMotors_);

    double leftInches = (leftDeg - prevLeftDeg_) * inchesPerDegree_;
    double rightInches = (rightDeg - prevRightDeg_) * inchesPerDegree_;

    prevLeftDeg_ = leftDeg;
    prevRightDeg_ = rightDeg;

    Matrix deltas(3, std::vector<double>(1, 0.0));   // 3x1: forward, lateral, turn
    deltas[0][0] = (leftInches + rightInches) / 2.0;
    deltas[1][0] = 0.0;
    deltas[2][0] = (rightInches - leftInches) / trackWidth_;
    return deltas;
}

void DriveEncoderLocalizer::update() {
    std::uint64_t now = pros::micros();
    double deltaTimeSec = (now - prevTimeMicros_) / 1'000'000.0;
    prevTimeMicros_ = now;

    Matrix robotDeltas = getRobotDeltas();
    double turn = robotDeltas[2][0];

    Matrix transformation(3, std::vector<double>(3, 0.0));
    if (std::abs(turn) < 0.001) {
        transformation[0][0] = 1.0 - (turn * turn) / 6.0;
        transformation[0][1] = -turn / 2.0;
        transformation[1][0] = turn / 2.0;
        transformation[1][1] = 1.0 - (turn * turn) / 6.0;
    } else {
        transformation[0][0] = std::sin(turn) / turn;
        transformation[0][1] = (std::cos(turn) - 1.0) / turn;
        transformation[1][0] = (1.0 - std::cos(turn)) / turn;
        transformation[1][1] = std::sin(turn) / turn;
    }
    transformation[2][2] = 1.0;

    Matrix globalDeltas = multiply(multiply(prevRotationMatrix_, transformation), robotDeltas);

    displacementPose_ = displacementPose_ + Pose(globalDeltas[0][0], globalDeltas[1][0], globalDeltas[2][0]);
    prevRotationMatrix_ = rotationMatrix(getPose().heading);
    totalHeading_ += globalDeltas[2][0];
    (void)deltaTimeSec; // keep this if you add velocity tracking later; unused for pose alone
}

Pose DriveEncoderLocalizer::getPose() const {
    return startPose_ + displacementPose_;
}

void DriveEncoderLocalizer::setStartPose(const Pose& pose) {
    startPose_ = pose;
}

void DriveEncoderLocalizer::setPose(const Pose& pose) {
    displacementPose_ = pose - startPose_;
    resetEncoders();
}