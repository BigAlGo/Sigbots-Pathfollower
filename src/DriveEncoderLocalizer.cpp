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


DriveEncoderLocalizer::DriveEncoderLocalizer(DriveLocalizerConstants c, Pose startPose)
    : leftMotors(std::make_unique<pros::MotorGroup>(c.leftMotorPorts)),
      rightMotors(std::make_unique<pros::MotorGroup>(c.rightMotorPorts)),
      trackWidth(c.trackWidthInches)
{
    inchesPerDegree = (c.wheelDiameterInches * M_PI) / 360.0 / c.externalGearRatio;
    startPose = startPose;
    displacementPose = Pose();
    prevRotationMatrix = rotationMatrix(0.0);
    resetEncoders();
    prevTimeMicros = pros::micros();
}

double DriveEncoderLocalizer::averagePosition(const pros::MotorGroup& group) const {
    std::vector<double> positions = group.get_position_all();
    double sum = 0.0;
    for (double p : positions) sum += p;
    return sum / positions.size();
}

void DriveEncoderLocalizer::resetEncoders() {
    leftMotors->tare_position_all();
    rightMotors->tare_position_all();
    prevLeftDeg = 0.0;
    prevRightDeg = 0.0;
}

Matrix DriveEncoderLocalizer::getRobotDeltas() {
    double leftDeg = averagePosition(*leftMotors);
    double rightDeg = averagePosition(*rightMotors);

    double leftInches = (leftDeg - prevLeftDeg) * inchesPerDegree;
    double rightInches = (rightDeg - prevRightDeg) * inchesPerDegree;

    prevLeftDeg = leftDeg;
    prevRightDeg = rightDeg;

    Matrix deltas(3, std::vector<double>(1, 0.0));   // 3x1: forward, lateral, turn
    deltas[0][0] = (leftInches + rightInches) / 2.0;
    deltas[1][0] = 0.0;
    deltas[2][0] = (rightInches - leftInches) / trackWidth;
    return deltas;
}

void DriveEncoderLocalizer::update() {
    std::uint64_t now = pros::micros();
    double deltaTimeSec = (now - prevTimeMicros) / 1'000'000.0;
    prevTimeMicros = now;

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

    Matrix globalDeltas = multiply(multiply(prevRotationMatrix, transformation), robotDeltas);

    displacementPose = displacementPose + Pose(globalDeltas[0][0], globalDeltas[1][0], globalDeltas[2][0]);
    prevRotationMatrix = rotationMatrix(getPose().heading);
    totalHeading += globalDeltas[2][0];
    (void)deltaTimeSec; // keep this if you add velocity tracking later; unused for pose alone
}

Pose DriveEncoderLocalizer::getPose() const {
    return startPose + displacementPose;
}

void DriveEncoderLocalizer::setStartPose(const Pose& pose) {
    startPose = pose;
}

void DriveEncoderLocalizer::setPose(const Pose& pose) {
    displacementPose = pose - startPose;
    resetEncoders();
}