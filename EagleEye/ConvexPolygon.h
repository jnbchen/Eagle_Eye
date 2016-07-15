
#ifndef _DerWeg_CONVEXPOLYGON_H__
#define _DerWeg_CONVEXPOLYGON_H__

#include "../Elementary/ThreadSafeLogging.h"
#include "../Elementary/Vec.h"

using namespace std;

namespace DerWeg {

class ConvexPolygon {
private:
    vector<Vec> points;

public:
    ConvexPolygon() {;}

    ConvexPolygon(vector<double> coords) {
        if (coords.size() % 2 != 0) {
            EOUT("Error: uneven number of coordinates in ConvexPolygon\n");
        } else {
            for (unsigned int i=0; i < coords.size(); i += 2) {
                points.push_back(Vec(coords[i], coords[i+1]));
            }
        }
    }

    bool isInside(Vec pos) {
        bool is_inside = true;
        // check for each edge, if pos is left of edge
        for (unsigned int i=0; i<points.size(); i++) {
            Vec& p_begin = points[i];
            Vec& p_end = points[(i+1)%points.size()];
            bool left = (pos - p_begin) * (p_end - p_begin).rotate_quarter() > 0;
            is_inside = is_inside && left;
        }
        return is_inside;
    }
};

} // Namespace

#endif // _DerWeg_CONVEXPOLYGON_H__
