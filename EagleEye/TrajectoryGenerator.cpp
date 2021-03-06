#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

#include "../Elementary/Vec.h"
#include "BezierCurve.h"
#include "Segment.h"
#include "TrafficLightBehaviour.h"
#include "ConvexPolygon.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstring>
#include <vector>
#include <map>


namespace DerWeg {

    // split a string at the characters defined by delimiter
    std::vector<std::string> split_string(const std::string& str,
                                          const std::string& delimiter) {
        std::vector<std::string> strings;

        std::string::size_type pos = 0;
        std::string::size_type prev = 0;
        while ((pos = str.find(delimiter, prev)) != std::string::npos) {
            strings.push_back(str.substr(prev, pos - prev));
            prev = pos + delimiter.size();
        }

        // To get the last substring (or only, if delimiter is not found)
        strings.push_back(str.substr(prev));

        return strings;
    }

  enum DrivingCommand {straight, right, left, out};

  /** TrajectoryGenerator */
  class TrajectoryGenerator : public KogmoThread {

  private:
    std::map<int, Segment> segments;
    int segment_index;
    int curve_index;
    std::map<int, std::map<DrivingCommand, int> > seg_lookup;

    std::list<DrivingCommand> commands;
    std::list<DrivingCommand> default_commands;

    double distance_threshold;
    double seeding_pts_per_meter;
    int qf_min_N;
    Angle angle_threshold;

    int precalculate_curvature_steps;
    double precalculate_curvature_distance;
    double precalculate_curvature_delta_s;
    double precalculate_curvature_seeding;

    double newton_tolerance;
    int newton_max_iter;
    double v_max;
    TrafficLightBehaviour tl_behaviour;

    double last_projection_parameter;

  public:
    TrajectoryGenerator () : seg_lookup(createMap()), last_projection_parameter(0) {}
    ~TrajectoryGenerator () {;}

    std::map<int, std::map<DrivingCommand, int> > createMap() {
        std::map<int, std::map<DrivingCommand, int> > seg_lookup;

        seg_lookup[1][straight] = 13;
        seg_lookup[2][straight] = 24;
        seg_lookup[3][straight] = 31;
        seg_lookup[4][straight] = 42;
        seg_lookup[1][right] = 14;
        seg_lookup[2][right] = 22;
        seg_lookup[3][right] = 33;
        seg_lookup[4][right] = 41;
        seg_lookup[1][left] = 11;
        seg_lookup[2][left] = 23;
        seg_lookup[3][left] = 32;
        seg_lookup[4][left] = 44;
        seg_lookup[1][out] = 15;
        seg_lookup[2][out] = 25;
        seg_lookup[3][out] = 35;
        seg_lookup[4][out] = 45;

        return seg_lookup;
    }

    void init(const ConfigReader& cfg) {
        LOUT("Exec TrajectoryGenerator init()" << std::endl);

        std::string file_path;
        cfg.get("TrajectoryGenerator::segments_file", file_path);

        std::vector<double> intersec_midpoint_temp;
        cfg.get("TrajectoryGenerator::intersection_midpoint", intersec_midpoint_temp);
        Vec intersec_midpoint(intersec_midpoint_temp[0], intersec_midpoint_temp[1]);

        // Read parameters
        cfg.get("LateralControl::newton_tolerance", newton_tolerance);
        cfg.get("LateralControl::newton_max_iter", newton_max_iter);

        cfg.get("TrajectoryGenerator::max_seg_distance", distance_threshold);
        cfg.get("TrajectoryGenerator::sppm", seeding_pts_per_meter);
        cfg.get("TrajectoryGenerator::qf_N", qf_min_N);

        cfg.get("TrajectoryGenerator::precalculate_curvature_steps", precalculate_curvature_steps);
        cfg.get("TrajectoryGenerator::precalculate_curvature_distance", precalculate_curvature_distance);
        cfg.get("TrajectoryGenerator::precalculate_curvature_seeding", precalculate_curvature_seeding);
        precalculate_curvature_delta_s = precalculate_curvature_distance / precalculate_curvature_steps;

        double degree_angle;
        cfg.get("TrajectoryGenerator::max_diff_degree", degree_angle);
        angle_threshold = Angle::deg_angle(degree_angle);

        cfg.get("LongitudinalControl::v_max", v_max);

        // Read driving commands
        std::vector<char> cmds;
        cfg.get("TrajectoryGenerator::driving_commands", cmds);
        for (size_t i = 0; i < cmds.size(); i++){
            switch (cmds[i]) {
                case 's': commands.push_back(straight); break;
                case 'l': commands.push_back(left); break;
                case 'r': commands.push_back(right); break;
                case 'o': commands.push_back(out); break;
                default : EOUT("Unknown driving command");
            }
        }

        std::vector<char> default_cmds;
        cfg.get("TrajectoryGenerator::default_commands", default_cmds);
        for (size_t i = 0; i < default_cmds.size(); i++){
            switch (default_cmds[i]) {
                case 's': default_commands.push_back(straight); break;
                case 'l': default_commands.push_back(left); break;
                case 'r': default_commands.push_back(right); break;
                case 'o': default_commands.push_back(out); break;
                default : EOUT("Unknown driving command");
            }
        }
        if (default_commands.size() == 0) {
            EOUT("No default command specified. Assume straight." << std::endl);
            default_commands.push_back(straight);
        }

        TrafficLightData tl;
        tl.state = none;
        tl.position = Vec(0,0);
        BBOARD->setTrafficLight(tl);

        tl_behaviour = TrafficLightBehaviour(cfg);

        std::ifstream input_file(file_path);
        std::string line;
        int segment_identifier = 0;

        // check if segments file could be opened
        if (!input_file.is_open()) {
            LOUT("ERROR: Could not open segments file: " << file_path << std::endl);
        }

        while (std::getline(input_file, line)) {

            // Delete trailing whitespace
            size_t endpos = line.find_last_not_of(" \n\r\t");
            line = (endpos == std::string::npos) ? "" : line.substr(0, endpos+1);

            // split lines by spaces
            std::vector<std::string> line_content =
                split_string(line, " ");

            // Delete leading whitespace for individuel strings of points
            for (unsigned int i = 0; i < line_content.size(); ++i) {
                size_t startpos = line_content[i].find_first_not_of(" \n\r\t");
                line_content[i] = (startpos == std::string::npos) ? "" :
                                  line_content[i].substr(startpos);
            }

            // distinguish lines: header, data and empty lines
            if (line_content[0] == "#" && line_content[1] == "segment" &&
                line_content.size() == 3) {
                // header line, start new segment
                segment_identifier = std::atoi(line_content[2].c_str());
                LOUT("Read Segment " << segment_identifier << std::endl);
            }
            else if (line_content.size() == 1 && line_content[0] == "") {
                // empty line, do nothing
            }
            else if (line_content.size() == 4 && segment_identifier != 0) {
                // data line, construct bezier curve, add to segment
                std::vector<Vec> points(4);
                for (int i = 0; i < 4; ++i) {
                    std::vector<std::string> coord = split_string(line_content[i], ",");
                    double x = std::atof(coord[0].c_str());
                    double y = std::atof(coord[1].c_str());
                    points[i] = Vec(x, y);
                }
                BezierCurve bc(points[0], points[1], points[2], points[3]);
                segments[segment_identifier].add(bc);
            }
            else
                EOUT("ERROR: something wrong with the segments file" << std::endl);
        }
        LOUT("Number of Segments: " << segments.size() << std::endl);

        for (std::map<int, Segment>::iterator iter = segments.begin(); iter != segments.end(); ++iter) {
            // get map value
            Segment& seg = iter->second;

            // Find segment position
            SegmentPosition seg_pos = seg.find_segment_position(intersec_midpoint, seeding_pts_per_meter, qf_min_N);
            // get map key
            //seg_pos.segment_id = iter->first;

            for (int i=0; i < seg.size(); i++) {
                BezierCurve& curve = seg.get(i);
                curve.behind_intersec = (i > seg_pos.curve_id);
                //LOUT("init_behind_intersec" << seg.get(i).behind_intersec<<"\n");
            }
        }
        LOUT("TrajectoryGenerator init() finished" << std::endl);
    }

    SegmentPosition find_start_position(const State state) {
/*
        EOUT("Position: " << state.position.x << ", " << state.position.y << std::endl);
        EOUT("Orientation: " << state.orientation.get_deg_180() << std::endl);
*/
        /* Find valid segment positions, close enough to position: */

        std::vector<SegmentPosition> valid_segments;
        for (std::map<int, Segment>::iterator iter = segments.begin(); iter != segments.end(); ++iter) {
            // get map value
            Segment seg = iter->second;

            // Find segment position
            SegmentPosition seg_pos = seg.find_segment_position(state.control_position, seeding_pts_per_meter, qf_min_N);
            // get map key
            seg_pos.segment_id = iter->first;

            // Check segment position for validity:
            // Distance has to be close enough, and the car's angle & curve angle must not differ too much
            BezierCurve curve = segments[seg_pos.segment_id].get(seg_pos.curve_id);
            Angle diff_angle = state.orientation - curve.orientation(seg_pos.curve_parameter);
/*
            EOUT("--- Segment " << seg_pos.segment_id << " ---------------------------------------------------" << std::endl);
            EOUT("Distance: " << seg_pos.min_distance << std::endl);
            EOUT("to point: " << curve(seg_pos.curve_parameter) << std::endl);
            EOUT("Diff-Angle: " << diff_angle.get_deg_180() << std::endl);
            EOUT("Angle_threshold: " << angle_threshold.get_deg_180() << std::endl);
            EOUT("in_range: " << diff_angle.in_between(-angle_threshold, angle_threshold) << std::endl);
*/
            if (seg_pos.min_distance < distance_threshold && diff_angle.in_between(-angle_threshold, angle_threshold)) {
                valid_segments.push_back(seg_pos);
                //EOUT("--- Segment " << seg_pos.segment_id << " ---------------------------------------------------" << std::endl);
            }
            /*
            else if (diff_angle.in_between(-angle_threshold, angle_threshold)) {
                LOUT("min dist: " << seg_pos.min_distance << std::endl);
            }
            */
        }

        if (valid_segments.size()==0) {
            EOUT("Error, no valid segments found" << std::endl);
        }
        // After filtering segments, decide logically which segment to drive on by segment_IDs

        /* Counting occurences: */

        // In there histograms we count how often each node occurs in the valid segments
        // as origin node (first digit) and destination node (second digit)
        // The number of occurences are saved to the corresponding indices
        // The index 0 holds the index out of {1,2,3,4} with the most occurences
        int origin_histogram [6] = {};
        int destination_histogram [6] = {};

        for (size_t i=0; i<valid_segments.size(); i++) {
            const int seg_id = valid_segments[i].segment_id;
            const int origin = seg_id / 10;
            const int destination = seg_id % 10;
            /*
            LOUT("seg_id" << seg_id << std::endl);
            LOUT("origin" << origin << std::endl);
            LOUT("destination" << destination << std::endl);
            */

            origin_histogram[origin] += 1;
            int max_origin_node = origin_histogram[0];
            if (origin_histogram[origin] > origin_histogram[max_origin_node]) {
                origin_histogram[0] = origin;
            }
            destination_histogram[destination] += 1;
            int max_destination_node = destination_histogram[0];
            if (destination != 5 && destination_histogram[destination] > destination_histogram[max_destination_node]) {
                destination_histogram[0] = destination;
            }
        }

        //LOUT("Histogramm finished" << std::endl);

        /* Decision logic: */

        //node, which occurs most as origin node
        int max_origin_node = origin_histogram[0];
        //node, which occurs most as destination node
        int max_destination_node = destination_histogram[0];


/*
        for (int i=0; i<=5; i++) {
            LOUT("origin histogram [" << i << "] = " <<origin_histogram[i] << std::endl);
        }

        LOUT("max_origin_node" << max_origin_node << std::endl);
        LOUT("origin_histogram" << origin_histogram[max_origin_node] << std::endl);
        LOUT("max_destination_node" << max_origin_node << std::endl);
        LOUT("destination_histogram" << destination_histogram[max_destination_node] << std::endl);
*/
        // true, if there are more occurences like 11,13,14 than 31,41,11
        bool is_after_node = (origin_histogram[max_origin_node] >= destination_histogram[max_destination_node]);
        // true if no node is occuring significantly often as origin or destination, but node 5 is occuring often as destination
        bool on_exit = (origin_histogram[max_origin_node] <= 1) && (destination_histogram[max_destination_node] <= 1) && (destination_histogram[5] >= 3);

        //LOUT("After node " << is_after_node << std::endl);
        //LOUT("On exit " << on_exit << std::endl);

        if (is_after_node && !on_exit) {
        // position is shortly after a node -> take next move_commmand into account, since there are multiple paths available
            int found_segment_id = get_next_segment(max_origin_node);
            for (size_t i=0; i<valid_segments.size(); i++) {
                if (valid_segments[i].segment_id == found_segment_id) {
                    return valid_segments[i];
                }
            }
            //The following case should not happen, just for robustness!
            EOUT("Error! Correct segment not in valid_segments!" << std::endl);
            return segments[found_segment_id].find_segment_position(state.control_position,seeding_pts_per_meter,qf_min_N);
        } else {
        // position is shortly before a node -> just take any valid path which ends at the max destination node
        // if starting on the exit ramp, take any segment ending at node 5
            if (on_exit) {
                max_destination_node = 5;
            }
            for (size_t i=0; i<valid_segments.size(); i++) {
                if (valid_segments[i].segment_id % 10 == max_destination_node) {
                    return valid_segments[i];
                }
            }
            //The following case should not happen, just for robustness!
            EOUT("Error! Could not find any valid segment ending at the mostly occuring node!" << std::endl);
            return valid_segments[0];
        }
    }

    int get_next_segment(const int starting_node) {
        // If command list is empty (or only one command left), refill it with the default command sequence
        while (commands.size() <= 1) {
            commands.insert(commands.end(), default_commands.begin(), default_commands.end());
        }

        DrivingCommand next_command = commands.front();
        commands.pop_front();

        return seg_lookup[starting_node][next_command];
    }

    void set_reference_trajectory(double tl_velocity, double curv_lookahead) {
        ReferenceTrajectory rt;
        rt.path = segments[segment_index].get(curve_index);
        rt.segment_id = segment_index;
        rt.v_max_tl = tl_velocity;
        rt.curvature_lookahead = curv_lookahead;
        //LOUT("Set curve with behind intersec = " << rt.path.behind_intersec
        //    << " or " << segments[segment_index].get(curve_index).behind_intersec << "\n");
        BBOARD->setReferenceTrajectory(rt);
    }

//    void write_curvature(State state) {
//        Vec pos = state.control_position;
//
//        Segment& seg = segments[segment_index];
//        BezierCurve& bc = seg.get(curve_index);
//        double t = bc.project(pos, 0.5, newton_tolerance, newton_max_iter);
//
//        //evaluate bezier curve and derivatives at the projection parameter
//        Vec f = bc(t);
//        Vec df = bc.prime(t);
//
//        Vec diff = pos - f;
//        double distance = diff.length();
//        //If the point is right of df, let the distance have a negative sign
//        if (diff * df.rotate_quarter() < 0) {
//            distance *= -1;
//        }
//        Angle diff_angle = state.orientation - bc.orientation(df);
//
//        SegmentPosition seg_pos;
//        seg_pos.curve_id = curve_index;
//        seg_pos.curve_parameter = t;
//
//        float min_vel = 0.1;
//
//        double delta_s = max(state.velocity_tire, min_vel) * precalculate_curvature_Ts * 1000; // times 1000 to convert to millimetres
//
//        vector<double> curvature = seg.precalculate_curvature(seg_pos, delta_s, precalculate_curvature_steps,
//                                                                precalculate_curvature_seeding, qf_min_N);
//
//        // Write calculations to txt file
//        ofstream datafile;
//        datafile.open("curvature.txt");
//
//        stringstream stream;
//
//        stream << precalculate_curvature_Ts << endl
//                 //<< state.velocity << endl
//                 << distance/1000 << endl
//                 << state.orientation.get_rad_pi() << endl
//                 << 0.5 * tan(state.steer.get_rad_pi()) << endl
//                 << (diff_angle + state.orientation).get_rad_pi() << endl;
//        for (int i = 0; i < curvature.size(); i++) {
//            stream << curvature[i] * 1000 << endl;
//        }
//
//        //LOUT(stream.str());
//
//        datafile << stream.str();
//
//        datafile.close();
//    }

    void execute() {

        BBOARD->setOnTrack(true);

        // Find starting position; has to be done here because while in init(), there will be no state written to the blackboard
        boost::this_thread::sleep(boost::posix_time::milliseconds(300));
        BBOARD->waitForState();

        // Selbstlokalisierung
        //SegmentPosition pos = find_start_position(BBOARD->getState());
        //segment_index = pos.segment_id;
        //curve_index = pos.curve_id;

        // Für Wettbewerb
        segment_index = 41;
        curve_index = 0;

        //segment_index = 25;
        //curve_index = 5;

        LOUT("Start execute with segment " << segment_index << std::endl);

        set_reference_trajectory(0,0);

        try{
            while (true) {
              if (BBOARD->getOnTrack()) {
                //BBOARD->waitForState();
                State state = BBOARD->getState();

                if (segments[segment_index].get(curve_index).reached_end(state.control_position)) {
                    if (curve_index < segments[segment_index].size() - 1) {
                        curve_index++;
                        last_projection_parameter = 0;
                    }
                    else {
                        curve_index = 0;
                        last_projection_parameter = 0;
                        int end_node = segment_index % 10;
                        if (end_node == 5) {
                            // Reached end of parcour
                            BBOARD->setOnTrack(false);
                            continue;
                        }
                        segment_index = get_next_segment(end_node);

                        // This takes into account, that the exit is right after the intersection
                        // in direction of node 4.
                        // E.g. if the car is on node 1, and gets the next commands r o
                        // Without the following block, the car would drive segment 14 and then 45,
                        // even if it could have fulfilled the commands "right" and "out" faster
                        // with the segment 15
                        if (segment_index % 10 == 4 && commands.size() > 0 && commands.front() == out) {
                            // Lets segment end at node 5 instead of node 4
                            segment_index += 1;
                            commands.pop_front();
                        }
                        // This avoids using segment 35, because this segment passes the intersection twice,
                        // so curve.behind_intersec is not well defined and thus can lead to problems with traffic lights etc.
                        // Solution: instead of driving segment 35, drive segment 31 and then segment 15
                        // condition is true, is current segment ends with node 3 and the upcoming command is "out"
                        else if (segment_index % 10 == 3 && commands.size() > 0 && commands.front() == out){
                            commands.push_front(straight);
                        }

                        LOUT("Start segment " << segment_index << std::endl);
                    }

                    LOUT("Curve index: " << curve_index << std::endl);
                        //<< "Behind intersection = " << segments[segment_index].get(curve_index).behind_intersec << std::endl);
                }
                else {
                    // do nothing
                }

                // Write precalculated curvature to txt file
                //write_curvature(state);

                //Segment Position
                Segment& seg = segments[segment_index];
                BezierCurve& bc = seg.get(curve_index);

                SegmentPosition seg_pos;
                seg_pos.curve_id = curve_index;
                seg_pos.segment_id = segment_index;
                last_projection_parameter = bc.project(state.control_position, last_projection_parameter, newton_tolerance, newton_max_iter);
                seg_pos.curve_parameter = last_projection_parameter;

                // Precalculated curvature
                vector<double> curvature = seg.precalculate_curvature(seg_pos, precalculate_curvature_delta_s, precalculate_curvature_steps,
                                                                        precalculate_curvature_seeding, qf_min_N);
                // Calculate mean curvature on next meter
                double mean_curv = accumulate( curvature.begin(), curvature.end(), 0.0) / curvature.size();


                // Check for traffic light and set behaviour accordingly
                double v_tl;
                  if (segments[segment_index].get(curve_index).behind_intersec) {
                    v_tl = v_max;
                  } else {
                    int tl_seg = segment_index;

                    //LOUT("CTRL: TRAJ_GEN: t = " << seg_pos.curve_parameter << endl);
                    seg_pos.min_distance = 0;

                    //BBOARD->waitForTrafficLight();
                    TrafficLightData tl_data = BBOARD->getTrafficLight();
                    //LOUT("tl state = " << tl_data.state << std::endl);
                    v_tl = tl_behaviour.calculate_max_velocity(tl_data, state.velocity_tire,
                                                            segments[tl_seg], seg_pos);
                    //LOUT("Set v = " << v << "\n");
                    if (v_tl < 0) {
                        //LOUT("TrajGen v = " << v <<"\n");
                    }
                  }
                set_reference_trajectory(v_tl, mean_curv);
              }
              boost::this_thread::sleep(boost::posix_time::milliseconds(20));
              boost::this_thread::interruption_point();
            }
        }catch(boost::thread_interrupted&){;}
    }

  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::TrajectoryGenerator> application ("TrajectoryGenerator");

}
