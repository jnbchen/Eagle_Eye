
#ifndef _DerWeg_BEZIERCURVE_H__
#define _DerWeg_BEZIERCURVE_H__


#include <math.h>
#include <limits>

#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"


namespace DerWeg {

    /** Small struct to enable returning two values in the seeding projection function */
    struct DistanceParameters {
        DistanceParameters(): t(0), distance(0) {}
        DistanceParameters(double s, double dist): t(s), distance(dist) {}
        double t;
        double distance;
    };

    /** Class to represent two dimensional Bezier Curves */
    class BezierCurve {
        private:
            Vec s;      ///< Coordinates of start point
            Vec c1;     ///< Coordinates of first control point
            Vec c2;     ///< Coordinates of second control point
            Vec e;      ///< Coordinates of end point
            double length; ///< Numerical approximation of the arclength
            int qf_used_N; ///< Number of intervals used for quadrature of arc_length

        public:
            friend class Segment;

            bool behind_intersec;   ///< Indicator if curve is behind or in front of intersection

            /** Constructors */
            BezierCurve();
            BezierCurve(Vec&, Vec&, Vec&, Vec&);
            BezierCurve(Vec&, Vec&, Vec&, Vec&, bool);

            /** Comparison Operators */
            bool operator== (const BezierCurve&) const;
            bool operator!= (const BezierCurve&) const;

            /** Evalutate Bezier Curve at postion t*/
            Vec operator() (const double t) const;

            /** Evalutate first derivative of Bezier Curve at postion t*/
            Vec prime(const double t) const;

            /** Evalutate second derivative of Bezier Curve at postion t*/
            Vec double_prime(const double t) const;

            /** Calculate nearest point on the Bezier Curve to a given position */
            double project(const Vec& position, const double newton_start,
                           const double newton_tol, const int max_iter) const;

            Angle orientation(const double t) const;
            Angle orientation(const Vec derivative) const;

            double curvature(const double t) const;
            double curvature(const double t, const Vec derivative) const;

            bool reached_end(const Vec& position) const;

            /** Numerically approximates the integral for calculating the arc length
                between parameters a and b, with a subdivision into N intervals.
                The used quadrature formula is the Simpson rule. */
            double arc_length(const double a, const double b, const int N);

            /** Numerically approximates the integral for calculating the arc length
                of the curve from 0 to 1, subdividing [0,1] at least into min_N intervals */
            double arc_length(const int min_N = 1);

            /** Seeds the curve with approximately the given seeding points per meter (assuming
            the units in bezier-points are millimeters).
            Returns the seeding parameter closest to the given position.
            min_N is the minimal number of subintervals for numerical integration of thee arc length. */
            DistanceParameters seeded_projection(const Vec position, const int seeding_pts_per_meter, const int min_N);


    };


} // namespace DerWeg

#endif // BEZIERCURVE_H__
