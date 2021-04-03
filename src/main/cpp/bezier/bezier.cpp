#include "bezier/bezier.h"

#include <cmath>

inline double sq (double v) { return v*v; }

namespace Bezier {

Point::Point evaluate (const CubicBezier &b, double t) {
    return (1-t) * evaluate(QuadraticBezier{b.p0, b.p1, b.p2}, t) +
            t * evaluate(QuadraticBezier{b.p1, b.p2, b.p3}, t);
}

Point::Point evaluate (const QuadraticBezier &b, double t) {
    return b.p1 + sq(1-t)*(b.p0 - b.p1) + sq(t)*(b.p2 - b.p1);
}

Derivatives evaluateDerivatives (const QuadraticBezier &b, double t) {
    double oneMinusT = 1 - t;
    Point::Point p1p0 = b.p1 - b.p0;
    Point::Point p2p1 = b.p2 - b.p1;
    return {
        2 * (oneMinusT * p1p0 + t * p2p1),
        2 * (p2p1 - p1p0)
    };
}

Derivatives evaluateDerivatives (const CubicBezier &b, double t) {
    QuadraticBezier qb1 = {b.p0, b.p1, b.p2};
    Derivatives d1 = evaluateDerivatives(qb1, t);
    Point::Point p1 = evaluate(qb1, t);

    QuadraticBezier qb2 = {b.p1, b.p2, b.p3};
    Derivatives d2 = evaluateDerivatives(qb2, t);
    Point::Point p2 = evaluate(qb2, t);

    double oneMinusT = 1 - t;
    
    return {
        oneMinusT * d1.firstDeriv - p1 + t * d2.firstDeriv + p2,
        oneMinusT * d1.secondDeriv - 2*d1.firstDeriv + t * d2.secondDeriv + 2*d2.firstDeriv
    };
}

CubicBezier split (CubicBezier *b, double t) {
    QuadraticBezier qb = approximate(b, t);
    double oneMinusT = 1 - t;
    Point::Point L1 = oneMinusT*qb.p0 + t*qb.p1;
    Point::Point L2 = oneMinusT*qb.p1 + t*qb.p2;
    Point::Point p = oneMinusT*L1 + t*L2;
    Point::Point e = b->p3;
    b->p1 = qb.p0;
    b->p2 = L1;
    b->p3 = p;
    return {p, L2, qb.p2, e};
}

QuadraticBezier split (QuadraticBezier *b, double t) {
    double oneMinusT = 1 - t;
    Point::Point L1 = oneMinusT*b->p0 + t*b->p1;
    Point::Point L2 = oneMinusT*b->p1 + t*b->p2;
    Point::Point p = oneMinusT*L1 + t*L2;
    Point::Point e = b->p2;
    b->p1 = L1;
    b->p2 = p;
    return {p, L2, e};
}

double curvature (const CubicBezier &b) {
    return (Point::distance(b.p0, b.p1) + Point::distance(b.p1, b.p2) + Point::distance(b.p2, b.p3)) / Point::distance(b.p0, b.p3);
}

double curvature (const QuadraticBezier &b) {
    return (Point::distance(b.p0, b.p1) + Point::distance(b.p1, b.p2)) / Point::distance(b.p0, b.p2);
}

// TODO optimize
std::vector<Sample> polylineApproximation (CubicBezier b, double maxCurvature, double maxLength, double startTime, double dTime) {
    if (curvature(b) <= maxCurvature && Point::distance(b.p0, b.p1) <= maxLength) {
        return {{b.p0, startTime}, {b.p3, startTime + dTime}};
    }

    dTime *= 0.5;
    std::vector<Sample> ret2 = polylineApproximation(split(&b, 0.5), maxCurvature, maxLength, startTime + dTime, dTime);
    std::vector<Sample> ret1 = polylineApproximation(b, maxCurvature, maxLength, startTime, dTime);

    ret1.reserve(ret1.size() + ret2.size() - 1);
    ret1.insert(ret1.end(), ret2.begin() + 1, ret2.end());
    return ret1;
}

// TODO optimize
std::vector<Sample> polylineApproximation (QuadraticBezier b, double maxCurvature, double startTime, double dTime) {
    if (curvature(b) <= maxCurvature) {
        return {{b.p0, startTime}, {b.p2, startTime + dTime}};
    }

    dTime *= 0.5;
    std::vector<Sample> ret2 = polylineApproximation(split(&b, 0.5), maxCurvature, startTime + dTime, dTime);
    std::vector<Sample> ret1 = polylineApproximation(b, maxCurvature, startTime, dTime);

    ret1.reserve(ret1.size() + ret2.size() - 1);
    ret1.insert(ret1.end(), ret2.begin() + 1, ret2.end());
    return ret1;
}

double getRadiusOfCurvature (Derivatives d) {
    return std::pow(Point::magnitude(d.firstDeriv), 3) / (d.firstDeriv.x * d.secondDeriv.y - d.firstDeriv.y * d.secondDeriv.x);
}

double getRadiusOfCurvature (const CubicBezier &b, double t) {
    return getRadiusOfCurvature(evaluateDerivatives(b, t));
}

double getRadiusOfCurvature (const QuadraticBezier &b, double t) {
    return getRadiusOfCurvature(evaluateDerivatives(b, t));
}

std::ostream& operator<< (std::ostream &os, const Sample &s) {
    os << s.p << " @ " << s.t;
    return os;
}

}
