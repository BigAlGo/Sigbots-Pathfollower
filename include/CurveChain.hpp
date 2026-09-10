#pragma once
#include "api.h"
#include "BezierCurve.hpp"
#include "DriveLocalizerConstants.hpp"

class CurveChain {
public:
    CurveChain();
    CurveChain(const std::vector<BezierCurve>& bezierCurves, DriveLocalizerConstants constants = DriveLocalizerConstants());
    CurveChain(const std::initializer_list<BezierCurve>& bezierCurves, DriveLocalizerConstants constants = DriveLocalizerConstants());

    void setConstants(DriveLocalizerConstants constants);

    BezierCurve getCurve(int t);
    int getLength();

    Pose pose;

private:
    std::vector<BezierCurve> bezierCurves;
    DriveLocalizerConstants constants;


};
