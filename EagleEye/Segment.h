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
            BezierCurve& get(const int i) {
                return curves.at(i);
            }

            int size() const {
                return curves.size();
            }

            // Calculate arc length distance from curve1, paramter value t1 to
            // curve2, paramter value t2
            // for the necessary quadrature are qf_N subintervals used per curve
            //
            // Returns -1 if invalid arguments
            double arc_length(int curve1, double t1, int curve2, double t2, int qf_N) {
                if (curve1 > curve2) {
                    return -1;
                }
                if (curve1 == curve2) {
                    if (t1 < t2) {
                        return get(curve1).arc_length(t1,t2,qf_N);
                    } else {
                        return -1;
                    }
                }

                double sum = 0;
                sum += get(curve1).arc_length(t1,1,qf_N);
                sum += get(curve2).arc_length(0,t2,qf_N);
                for (int i = curve1 + 1; i < curve2; i++) {
                    sum += get(i).arc_length(qf_N);
                }
                return sum;
            }

            /** Given a position vector, find the closest point on the segment to this position.
            This is done via seeding all curves on the segment and simply look out for the minimum distance.
            The parameter seeding_pts_per_meter is used to calculate how many seeding point are used on each curve (via arc length).
            max_distance is the maximum distance allowed from the position to the segment.
            min_N is the minimum number of quadrature subintervals for calculation of arc length.*/
            SegmentPosition find_segment_position(const Vec position, const int seeding_pts_per_meter, const int min_N) {
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
