#ifndef _DerWeg_COVMAT_H__
#define _DerWeg_COVMAT_H__


#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"
#include <vector>
#include <cmath>

namespace DerWeg{

/** Simple implemenation of a 2x2 Matrix */
class CovMat {
    private:
        double a, b, c, d;
        /* C = (a, b;
                c, d)
        */

    public:
        CovMat() {}
        CovMat(const double a0, const double b0,
               const double c0, const double d0) :
            a(a0), b(b0), c(c0), d(d0) {}
        CovMat(std::vector<double> v) :
            a(v[0]), b(v[1]), c(v[2]), d(v[3]) {}
        ~CovMat() {}

        CovMat operator+ (const CovMat& C) const {
            // Matrix-Matrix-Sum
            return CovMat(a+C.a, b+C.b, c+C.c, d+C.d);
        }

        CovMat operator* (const CovMat& C) {
            // Matrix-Matrix-Product
            return CovMat(a*C.a + b*C.c, a*C.b + b*C.d,
                           c*C.a + d*C.c, c*C.b + d*C.d);
        }

        Vec operator* (const Vec& v) const {
            // Matrix-Vector-Product
            return Vec(a*v.x + b*v.y, c*v.x + d*v.y);
        }

        double det() const {
            // Determinant
            return a*d - b*c;
        }

        CovMat inv() const {
            // Inverse
            double det = this->det();
            return CovMat(d/det, -b/det, -c/det, a/det);
            }

        CovMat t() const {
            // Transpose
            return CovMat(a, c, b, d);
        }

};

}

#endif
