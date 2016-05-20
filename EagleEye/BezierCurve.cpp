
#include <cmath>
#include "BezierCurve.h"

using namespace DerWeg;

BezierCurve::BezierCurve() :
    s(Vec(0,0)), c1(Vec(0,0)), c2(Vec(0,0)), e(Vec(0,0)) {;}

BezierCurve::BezierCurve(Vec& start, Vec& control1, Vec& control2, Vec& end)
    : s(start), c1(control1), c2(control2), e(end) {;}

bool BezierCurve::operator== (const BezierCurve& bc) const {
    return s==bc.s && c1==bc.c1 && c2==bc.c2 && e==bc.e;
}

bool BezierCurve::operator!= (const BezierCurve& bc) const {
    return !(*this == bc);
}

Vec BezierCurve::operator() (const double t) const {
    return std::pow(1-t, 3) * s + 3*t*std::pow(1-t, 2) * c1 +
            3*std::pow(t, 2)*(1-t) * c2 + std::pow(t, 3) * e;
}

Vec BezierCurve::prime(const double t) const {
    return 3*std::pow(1-t, 2) * (c1-s) + 6*t*(1-t) * (c2-c1) +
            3*std::pow(t, 2) * (e-c2);
}

Vec BezierCurve::double_prime(const double t) const {
    return 6*(1-t) * (c2 - 2*c1 + s) + 6*t * (e - 2*c2 + c1);
}

double BezierCurve::project(const Vec& position, const double newton_start) const {
    // asdf
    // return t;
    return 0;
}
