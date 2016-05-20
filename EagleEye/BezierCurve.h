
#ifndef _DerWeg_BEZIERCURVE_H__
#define _DerWeg_BEZIERCURVE_H__

#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"


namespace DerWeg {

    /** Class to represent two dimensional Bezier Curves */
    class BezierCurve {
        private:
            Vec s;      ///< Coordinates of start point
            Vec c1;     ///< Coordinates of first control point
            Vec c2;     ///< Coordinates of second control point
            Vec e;      ///< Coordinates of end point

        public:
            /** Constructors */
            BezierCurve();
            BezierCurve(Vec&, Vec&, Vec&, Vec&);

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
            double project(const Vec& position, const double newton_start = 0) const;

    };


} // namespace DerWeg

#endif // BEZIERCURVE_H__
