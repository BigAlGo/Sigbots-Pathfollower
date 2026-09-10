#include "PIDFController.hpp"

PIDFController::PIDFController() {
    coefficients = PIDFCoefficients();
}

PIDFController::PIDFController(PIDFCoefficients set) {
    coefficients = set;
    reset();
}

double PIDFController::run() {
    return error * P() + errorDerivative * D() + errorIntegral * I() + feedForwardInput * F();
}

void PIDFController::updatePosition(double position) {
    this->position = position;
    previousError = error;
    error = targetPosition - this->position;

    long long currentNano = getSystemNanoTime();
    deltaTimeNano = currentNano - previousUpdateTimeNano;
    previousUpdateTimeNano = currentNano;

    double deltaTimeSec = deltaTimeNano / 1e9;
    errorIntegral += error * deltaTimeSec;
    errorDerivative = (error - previousError) / deltaTimeSec;
}

void PIDFController::updateError(double error) {
    previousError = this->error;
    this->error = error;
    
    long long currentNano = getSystemNanoTime();
    deltaTimeNano = currentNano - previousUpdateTimeNano;
    previousUpdateTimeNano = currentNano;

    double deltaTimeSec = deltaTimeNano / 1e9;
    errorIntegral += error * deltaTimeSec;
    errorDerivative = (error - previousError) / deltaTimeSec;
}

void PIDFController::updateFeedForwardInput(double input) {
    feedForwardInput = input;
}

void PIDFController::reset() {
    previousError = 0;
    error = 0;
    position = 0;
    targetPosition = 0;
    errorIntegral = 0;
    errorDerivative = 0;
    previousUpdateTimeNano = getSystemNanoTime();
}

void PIDFController::setTargetPosition(double set) {
    targetPosition = set;
}

double PIDFController::getTargetPosition() const {
    return targetPosition;
}

void PIDFController::setCoefficients(PIDFCoefficients set) {
    coefficients = set;
}

PIDFCoefficients PIDFController::getCoefficients() const {
    return coefficients;
}
\

// Getter and setter methods
void PIDFController::setP(double set) { coefficients.P = set; }
double PIDFController::P() const { return coefficients.P; }

void PIDFController::setI(double set) { coefficients.I = set; }
double PIDFController::I() const { return coefficients.I; }

void PIDFController::setD(double set) { coefficients.D = set; }
double PIDFController::D() const { return coefficients.D; }

void PIDFController::setF(double set) { coefficients.F = set; }
double PIDFController::F() const { return coefficients.F; }

double PIDFController::getError() const { return error; }
double PIDFController::getErrorDerivative() const { return errorDerivative; }

long long PIDFController::getSystemNanoTime() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}
