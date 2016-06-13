#ifndef _DerWeg_SEGMENT_H__
#define _DerWeg_SEGMENT_H__

#include "BezierCurve.h"

namespace DerWeg{

    /** Struct to represent the position within a segment, and the minimum distance found to the segment */
    struct SegmentPosition {
        //SegmentPosition():segment_id(0), curve_id(0), curve_parameter(0) {}
        //SegmentPosition(int seg_id, int bc_id, double t):
        //                segment_id(seg_id), curve_id(bc_id), curve_parameter(t) {}
        int segment_id;
        int curve_id;
        double curve_parameter;
        //Vec projected_point;
        double min_distance;
    };

    /** Class to represent segments consisting of bezier curves */
    class Segment {
        private:
            std::vector<BezierCurve> curves;    ///< list of bezier curves in segment

        public:
            /** Add bezier curve at end of segment */
            void add(const BezierCurve& bc) {
                curves.push_back(bc);
            }

            /** Get i-th bezier curve of the segment */
            BezierCurve get(const int i) const {
                return curves.at(i);
            }

            int size() const {
                return curves.size();
            }

            /** Find current bezier curve */
//            int find(const State& state, const int start_index) const {
//                /*
//                Increase index as long as position lies "underneath" the line
//                rectangular to the curve at the endpoint of a bezier curve.
//                */
//                int index = start_index;
//                while ((curves[index].e - curves[index].c2) *
//                        (state.position - curves[index].e) >= 0) {
//                    index++;
//                }
//                return index;
//            }

            /** Given a position vector, find the closest point on the segment to this position.
            This is done via seeding all curves on the segment and simply look out for the minimum distance.
            The parameter seeding_pts_per_meter is used to calculate how many seeding point are used on each curve (via arc length).
            max_distance is the maximum distance allowed from the position to the segment.
            min_N is the minimum number of quadrature subintervals for calculation of arc length.*/
            SegmentPosition find_segment_position(const Vec position, const int seeding_pts_per_meter, const int min_N) const {
                SegmentPosition seg_pos;
                seg_pos.min_distance = std::numeric_limits<double>::max();
                for (unsigned int i=0; i<curves.size(); i++) {
                    DistanceParameters p = this->get(i).seeded_projection(position, seeding_pts_per_meter, min_N);
                    if (p.distance < seg_pos.min_distance) {
                        seg_pos.curve_id = i;
                        seg_pos.curve_parameter = p.t;
                        seg_pos.min_distance = p.distance;
                    }
                }
                return seg_pos;
            }


    };

}

#endif
