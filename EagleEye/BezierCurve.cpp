
#include <math.h>
#include <limits>

#include "../Elementary/ThreadSafeLogging.h"

#include "BezierCurve.h"

using namespace DerWeg;

BezierCurve::BezierCurve() :
    s(Vec(0,0)), c1(Vec(0,0)), c2(Vec(0,0)), e(Vec(0,0)), length(0), qf_used_N(0), behind_intersec(false) {}

BezierCurve::BezierCurve(Vec& start, Vec& control1, Vec& control2, Vec& end)
    : s(start), c1(control1), c2(control2), e(end), length(0), qf_used_N(0), behind_intersec(false) {}

BezierCurve::BezierCurve(Vec& start, Vec& control1, Vec& control2, Vec& end, bool bh_is)
    : s(start), c1(control1), c2(control2), e(end), length(0), qf_used_N(0), behind_intersec(bh_is) {}

bool BezierCurve::operator== (const BezierCurve& bc) const {
    return s==bc.s && c1==bc.c1 && c2==bc.c2 && e==bc.e && behind_intersec==bc.behind_intersec;
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

Angle BezierCurve::orientation(const double t) const {
    return orientation(this->prime(t));
}

Angle BezierCurve::orientation(const Vec derivative) const {
//    double phi;
//    if (derivative.x != 0 && abs(derivative.y/derivative.x) < 1) {
//        //atan2 takes the direction into account
//        phi = atan2(derivative.y, derivative.x);
//    } else {
//        //avoid singularities by rotating coordinate system for 2nd and 4th quadrant
//        phi = M_PI/2 + atan2(-derivative.x, derivative.y);
//    }
//    return Angle::rad_angle(phi);
    return derivative.angle();
}

double BezierCurve::curvature(const double t) const {
    return curvature(t, this->prime(t));
}

double BezierCurve::curvature(const double t, const Vec derivative) const {
    Vec ddf = double_prime(t);

    double curvature_numerator = ddf.y * derivative.x - ddf.x * derivative.y;
    double curvature_denominator = pow(derivative.squared_length(), 3.0/2);
    return curvature_numerator / curvature_denominator;
}

double BezierCurve::arc_length(const double a, const double b, const int N) {
    //Divide [a,b] into N subintervals of length h
    double h = (b - a)/N;
    if (h < 0) {
        //LOUT("Error in quadrature: h = " << h << std::endl);
    }
    double length = 0; //sum up length

    // function evaluation at left, middle, and right integration point of subintervals
    // For calculating the arc length, take the norm of the derivative vector
    double left, middle, right = this->prime(a).length();

    for (int i=0; i<N; i++) {
        //Calculate function values for simpson rule
        // Don't calculate the left value subinterval, because it is the same as the right value of
        // the previous interval --> Save computation time
        left = right;
        middle = this->prime(a + (i + 0.5)*h ).length();
        right = this->prime(a + (i + 1)*h ).length();

        // Simpson rule
        length += h * (left + 4*middle + right) / 6;
    }
    if(length < -100) {
        // only if more than 10 cm
        LOUT("Error in quadrature formula, l = "<<length<<std::endl);
    }
    return length;
}

double BezierCurve::arc_length(const int min_N) {
    // Only calculate arc length if it has not been calculated before, or if it has only been calculated
    // with less subintervals than required
    if (length == 0 || min_N > qf_used_N) {
        length = arc_length(0,1,min_N);
        qf_used_N = min_N;
    }
    return length;
}

DistanceParameters BezierCurve::seeded_projection(const Vec position, const int seeding_pts_per_meter, const int min_N) {
    double min_t = 0;
    double min_distance = std::numeric_limits<double>::max();

    int seeding_pts = (int) ceil(seeding_pts_per_meter * arc_length(min_N) / 1000);
    //LOUT("\tSeeding points = " << seeding_pts << "\n");
    double seeded_length = 1.0/seeding_pts;

    for (int i=0; i<seeding_pts; i++) {
        double seed_t = i * seeded_length;
        double distance = (position - this->operator()(seed_t)).length();
        if (distance < min_distance) {
            min_t = seed_t;
            min_distance = distance;
        }
    }
    DistanceParameters param(min_t, min_distance);
    return param;
}

bool BezierCurve::reached_end(const Vec& position) const {
    return ((e - c2) * (position - e) >= 0);
}

