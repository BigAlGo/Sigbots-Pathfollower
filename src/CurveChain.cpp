#include "CurveChain.hpp"

CurveChain::CurveChain() {
    this->bezierCurves;
    this->constants;
}

CurveChain::CurveChain(const std::initializer_list<BezierCurve>& bezierCurves, DriveLocalizerConstants constants) {
    this->bezierCurves = bezierCurves;
    this->constants = constants;
}

CurveChain::CurveChain(const std::vector<BezierCurve>& bezierCurves, DriveLocalizerConstants constants) {
    this->bezierCurves = bezierCurves;
    this->constants = constants;
}

void CurveChain::setConstants(DriveLocalizerConstants constants) {
    this->constants = constants;
} 

BezierCurve CurveChain::getCurve(int i) {
    return bezierCurves.at(i);
}

int CurveChain::getLength() {
    return bezierCurves.size();
}