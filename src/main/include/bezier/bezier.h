#pragma once

#include <vector>

#include "point.h"

namespace Bezier {

struct QuadraticBezier {
    Point::Point p0, p1, p2;
};

struct CubicBezier {
    Point::Point p0, p1, p2, p3;
};

struct Derivatives {
    Point::Point firstDeriv, secondDeriv;
};

struct Sample {
    Point::Point p;
    double t;
};

inline QuadraticBezier approximate (const CubicBezier &b, double t) {
    double oneMinusT = 1 - t;
    return {oneMinusT * b.p0 + t * b.p1, oneMinusT * b.p1 + t * b.p2, oneMinusT * b.p2 + t * b.p3};
}

inline QuadraticBezier approximate (const CubicBezier *b, double t) {
    double oneMinusT = 1 - t;
    return {oneMinusT * b->p0 + t * b->p1, oneMinusT * b->p1 + t * b->p2, oneMinusT * b->p2 + t * b->p3};
}

Point::Point evaluate(const CubicBezier &b, double t);
Point::Point evaluate(const QuadraticBezier &b, double t);

Derivatives evaluateDerivatives(const CubicBezier &b, double t);
Derivatives evaluateDerivatives(const QuadraticBezier &b, double t);

CubicBezier split(CubicBezier* b, double t);
QuadraticBezier split(QuadraticBezier* b, double t);

std::vector<Sample> polylineApproximation(CubicBezier b, double curvature = 1.001, double maxLength = 1.0e10, double startTime = 0, double endTime = 1);
std::vector<Sample> polylineApproximation(QuadraticBezier b, double curvature = 1.005, double startTime = 0, double endTime = 1);

double getRadiusOfCurvature(Derivatives d);
double getRadiusOfCurvature(const CubicBezier &b, double t);
double getRadiusOfCurvature(const QuadraticBezier &b, double t);

std::ostream& operator<<(std::ostream &os, const Sample &s);

}
