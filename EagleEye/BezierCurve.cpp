
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

double BezierCurve::project(const Vec& position, const double newton_start,
                            const double newton_tol, const int max_iter) const {
    double t = newton_start, delta_t;

    //evaluation of bezier curve at parameter t, and its derivatives
    // and difference vector of current f(t) and position
    Vec f, df, ddf, diff;

    //first and second derivative of the squared distance from position to curve at parameter t (modulo factor of 2)
    double sqdistance_prime = newton_tol + 1, sqdistance_2prime;

    for (int n=0; n < max_iter and sqdistance_prime > newton_tol; n++){
        f = this->operator()(t);
        df = this->prime(t);
        ddf = this->double_prime(t);
        diff = f - position;

        //Derive formulas for derivatives of squared distance by hand
        sqdistance_prime = diff * df;
        sqdistance_2prime = diff * ddf + df.squared_length();

        if (sqdistance_2prime > 0) {
            // newton-step only if second derivative is positive, otherwise the quadratic approximation
            // is maximized
            delta_t = - sqdistance_prime / sqdistance_2prime;
        } else {
            // if the second derivative is not positive, do a gradient step to get into convergence environment
            delta_t = - sqdistance_prime;
        }
        //update t with stepsize = 1
        t += delta_t;
    }

    return t;
}
