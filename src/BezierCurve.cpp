#include "BezierCurve.h"
#include "Pose.h"
using Matrix = std::vector<std::vector<double>>;

// creates pascal's triangle in a n+1 x n+1 matrix
static Matrix pascalTriangle(int n) {
    Matrix C(n + 1, std::vector<double>(n+1, 0.0));
    for (int m = 0; m <= n; ++m) {
        C[m][0] = 1.0;
        C[m][m] = 1.0;
        for (int j = 1; j < m; ++j) {
            C[m][j] = C[m - 1][j - 1] + C[m - 1][j];
        }
    }
    return C;
}

BezierCurve::BezierCurve(const std::vector<Pose>& controlPoints, int lutResolution) {
    if (controlPoints.size() < 3) {
        throw std::invalid_argument("BezierCurve needs 3 or more control points");
    }
    buildPowerBasis(controlPoints);
    buildDerivative();
    buildArcLengthTable(lutResolution);
}

BezierCurve::BezierCurve(int lutResolution) {
    std::vector<Pose> controlPoints = {Pose(), Pose(), Pose()};
    buildPowerBasis(controlPoints);
    buildDerivative();
    buildArcLengthTable(lutResolution);

}

void BezierCurve::buildPowerBasis(const std::vector<Pose>& pts) {
    degree_ = static_cast<int>(pts.size()) - 1;
    auto C = pascalTriangle(degree_);
    xCoeffs_.assign(degree_ + 1, 0.0);
    yCoeffs_.assign(degree_ + 1, 0.0);

    for (int k = 0; k <= degree_; ++k) {
        double sumX = 0.0;
        double sumY = 0.0;
        for (int i = 0; i <= k; ++i) {
            double sign = ((k - i) % 2 == 0) ? 1.0 : -1.0;
            double coeff = C[degree_][i] * C[degree_ - i][k - i] * sign;
            sumX += coeff * pts[i].x;
            sumY += coeff * pts[i].y;
        }
        xCoeffs_[k] = sumX;
        yCoeffs_[k] = sumY;
    }
}

double BezierCurve::evalPoly(const std::vector<double>& coeffs, double t) const {
    double result = 0.0;
    for (int i = static_cast<int>(coeffs.size()) - 1; i >= 0; --i) {
        result = result * t + coeffs[i];
    }
    return result;
}

void BezierCurve::buildDerivative() {
    dxCoeffs_.assign(degree_, 0.0);
    dyCoeffs_.assign(degree_, 0.0);
    for (int k = 0; k < degree_; ++k) {
        dxCoeffs_[k] = (k + 1) * xCoeffs_[k + 1];
        dyCoeffs_[k] = (k + 1) * yCoeffs_[k + 1];
    }
}

double BezierCurve::speedAt(double t) const {
    double dx = evalPoly(dxCoeffs_, t);
    double dy = evalPoly(dyCoeffs_, t);
    return std::sqrt(dx * dx + dy * dy);
}

Pose BezierCurve::getPoseAtT(double t) const {
    double x = evalPoly(xCoeffs_, t);
    double y = evalPoly(yCoeffs_, t);
    double dx = evalPoly(dxCoeffs_, t);
    double dy = evalPoly(dyCoeffs_, t);
    return Pose(x, y, std::atan2(dy, dx));
}

double BezierCurve::arcLength(double tStart, double tEnd, int subdivisions) const {
    double h = (tEnd - tStart) / subdivisions;
    double sum = speedAt(tStart) + speedAt(tEnd);
    for (int i = 1; i < subdivisions; ++i) {
        double t = tStart + i * h;
        sum += speedAt(t) * (i % 2 == 0 ? 2.0 : 4.0);
    }
    return sum * h / 3.0;
}

void BezierCurve::buildArcLengthTable(int resolution) {
    tSamples_.resize(resolution + 1);
    sSamples_.resize(resolution + 1);
    tSamples_[0] = 0.0;
    sSamples_[0] = 0.0;
    for (int i = 1; i <= resolution; ++i) {
        double t =  (i) / resolution;
        tSamples_[i] = t;
        sSamples_[i] = sSamples_[i - 1] + arcLength(tSamples_[i - 1], t, 4);
    }
    totalLength_ = sSamples_.back();
}

double BezierCurve::getTFromDistance(double distance) const {
    distance = std::clamp(distance, 0.0, totalLength_);

    auto it = std::lower_bound(sSamples_.begin(), sSamples_.end(), distance);
    size_t idx = std::distance(sSamples_.begin(), it);
    if (idx == 0) return 0.0;
    if (idx >= sSamples_.size()) return 1.0;

    double s0 = sSamples_[idx - 1], s1 = sSamples_[idx];
    double t0 = tSamples_[idx - 1], t1 = tSamples_[idx];

    if (s1 - s0 == 0.0) return t0;

    double t = t0 + (distance - s0) / (s1 - s0) * (t1 - t0);   // initial guess

    for (int iter = 0; iter < 4; ++iter) {
        double error = arcLength(0.0, t, 8) - distance;
        double speed = speedAt(t);
        if (speed < 1e-9) break;                // guards against a cusp/stationary point
        double next = std::clamp(t - error / speed, 0.0, 1.0);
        if (std::abs(next - t) < 1e-12) { t = next; break; }
        t = next;
    }
    return t;
}

double BezierCurve::getDistanceFromT(double t) const {
    return (totalLength_ * t);
}

Pose BezierCurve::getPoseAtDistance(double distance) const {
    return getPoseAtT(getTFromDistance(distance));
}