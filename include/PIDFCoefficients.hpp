#pragma once

struct PIDFCoefficients {
    double P;
    double I;
    double D;
    double F;

    // Default Constructor
    PIDFCoefficients() : P(0.0), I(0.0), D(0.0), F(0.0) {}

    PIDFCoefficients(double p, double i, double d, double f) 
        : P(p), I(i), D(d), F(f) {}
};