#pragma once

#include "api.h"
#include "PIDFCoefficients.hpp"

class PIDFController {
public:
    // Constructor accepts the static coefficients directly
    PIDFController();
    PIDFController(PIDFCoefficients set);

    // Core PIDF operations
    double run();
    void updatePosition(double position);
    void updateError(double error);
    void updateFeedForwardInput(double input);
    void reset();

    // Getters and Setters
    void setTargetPosition(double set);
    double getTargetPosition() const;
    void setCoefficients(PIDFCoefficients set);
    PIDFCoefficients getCoefficients() const;

    void setP(double set);
    double P() const;
    void setI(double set);
    double I() const;
    void setD(double set);
    double D() const;
    void setF(double set);
    double F() const;

    double getError() const;
    double getErrorDerivative() const;

private:
    PIDFCoefficients coefficients;

    double previousError;
    double error;
    double position;
    double targetPosition;
    double errorIntegral;
    double errorDerivative;
    double feedForwardInput;

    long long previousUpdateTimeNano;
    long long deltaTimeNano;

    // Helper method to fetch current time in nanoseconds
    long long getSystemNanoTime() const;
};
