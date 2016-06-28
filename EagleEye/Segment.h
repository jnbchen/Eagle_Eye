#ifndef _DerWeg_SEGMENT_H__
#define _DerWeg_SEGMENT_H__

#include "../Blackboard/Blackboard.h"
#include "../Elementary/ThreadSafeLogging.h"
#include "BezierCurve.h"
#include <sstream>

using namespace std;

namespace DerWeg{

    typedef vector <pair <double,double> > dTable;

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

        /**
        Advances the segment position by the given delta_t
        Does not guerantee that the curve_id stays within the segment!
        */
        int advance(double delta_t) {
            curve_parameter += delta_t;
            while (curve_parameter > 1) {
                curve_id += 1;
                curve_parameter -= 1;
            }
            //doesn't tell anything after advancing..
            min_distance = -1;

            return curve_id;
        }
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
                        return -2;
                    }
                }

                double sum = 0;
                //LOUT("CTRL: Segment.arc_length() 1\n");
                //LOUT("CTRL: t1, t2 = " << t1 << ", " << t2 << std::endl);
                sum += get(curve1).arc_length(t1,1,qf_N);
                sum += get(curve2).arc_length(0,t2,qf_N);
                for (int i = curve1 + 1; i < curve2; i++) {
                    sum += get(i).arc_length(qf_N);
                }
                //LOUT("CTRL: Segment.arc_length() 2\n");
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
                for (unsigned int i=0; i < curves.size(); i++) {
                    DistanceParameters p = this->get(i).seeded_projection(position, seeding_pts_per_meter, min_N);
                    //LOUT("c_id = " << i << "\nt = " << p.t << "\nd = " << p.distance << "\n");
                    if (p.distance < seg_pos.min_distance) {
                        seg_pos.curve_id = i;
                        seg_pos.curve_parameter = p.t;
                        seg_pos.min_distance = p.distance;
                    }
                }
                return seg_pos;
            }

            vector<double> precalculate_curvature(SegmentPosition seg_pos, double delta_s, int N_steps, double seeding_distance, int min_qf) {
                dTable sample_curvatures;

                double total_length = delta_s * N_steps;
                double length = 0;

                // sample curvatures
                do {
                    pair<double,double> new_entry;
                    new_entry.first = length;
                    new_entry.second = get(seg_pos.curve_id).curvature(seg_pos.curve_parameter);
                    sample_curvatures.push_back(new_entry);

                    std::stringstream pos;
                    Vec point = get(seg_pos.curve_id)(seg_pos.curve_parameter);
                    pos << "thick black dot "
                        << point.x << " " << point.y << std::endl;
                    BBOARD->addPlotCommand(pos.str());

                    pair<int,double> old_pos(seg_pos.curve_id, seg_pos.curve_parameter);
                    seg_pos.advance(seeding_distance);

                    if(seg_pos.curve_id < size()) {
                        // if still within curve
                        length += arc_length(old_pos.first, old_pos.second, seg_pos.curve_id, seg_pos.curve_parameter, min_qf);
                    } else {
                        // if out of curve
                        pair<double,double> new_entry;
                        new_entry.first = total_length + 1;
                        new_entry.second = 0;
                        sample_curvatures.push_back(new_entry);
                    }
                } while (sample_curvatures[sample_curvatures.size()-1].first < total_length);

                vector<double> res;
                //add curvature at current position
                res.push_back(sample_curvatures[0].second);

                int index = 0;
                for (int k=1; k<= N_steps; k++) {
                    double s = k * delta_s;
                    while (sample_curvatures[index].first < s) {
                        index++;
                    }
                    //linear interpolation:
                    double percentage = (s - sample_curvatures[index - 1].first) /
                                        (sample_curvatures[index].first - sample_curvatures[index - 1].first);
                    if (!(0 <= percentage && percentage <= 1)) {
                        LOUT("Error in precalculation of curvature \n");
                    }
                    double curv_at_s = percentage * sample_curvatures[index - 1].second + (1 - percentage) * sample_curvatures[index].second;
                    res.push_back(curv_at_s);
                }
                return res;
            }


    };

}

#endif
