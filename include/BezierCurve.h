#pragma once
#include "api.h"
#include "Pose.h"

class BezierCurve {
public:
    // Call with BezierCurve({Pose(), Pose(), Pose()})
    BezierCurve(const std::vector<Pose>& controlPoints, int lutResolution = 500);
    BezierCurve(int lutResolution = 500);

    Pose getPoseAtT(double t) const;
    Pose getPoseAtDistance(double distance) const;
    double getTFromDistance(double distance) const;
    double getDistanceFromT(double t) const;
    double length() const { return totalLength_; }

private:
    int degree_;
    std::vector<double> xCoeffs_, yCoeffs_;    // power-basis polynomial: xCoeffs_[k] is coeff of t^k
    std::vector<double> dxCoeffs_, dyCoeffs_;  // derivative polynomial

    double totalLength_;
    std::vector<double> tSamples_, sSamples_;  // lookup table: distance-at-t

    void buildPowerBasis(const std::vector<Pose>& pts);
    void buildDerivative();
    void buildArcLengthTable(int resolution);

    double evalPoly(const std::vector<double>& coeffs, double t) const;
    double speedAt(double t) const;
    double arcLength(double tStart, double tEnd, int subdivisions) const;
};