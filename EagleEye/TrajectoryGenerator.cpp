#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

#include "../Elementary/Vec.h"
#include "BezierCurve.h"

#include <iostream>
#include <fstream>
#include <string>
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
            int find(const State& state, const int start_index) const {
                /*
                Increase index as long as position lies "underneath" the line
                rectangular to the curve at the endpoint of a bezier curve.
                */
                int index = start_index;
                while ((curves[index].e - curves[index].c2) *
                        (state.position - curves[index].e) >= 0) {
                    index++;
                }
                return index;
            }
    };

  enum DrivingCommand {straight, right, left, out};

  /** TrajectoryGenerator */
  class TrajectoryGenerator : public KogmoThread {

  private:
    std::map<int, Segment> segments;
    int segment_index;
    int curve_index;
    std::map<int, std::map<DrivingCommand, int> > seg_lookup;
    std::list<DrivingCommand> commands;

  public:
    TrajectoryGenerator () : seg_lookup(createMap()) {}
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
        seg_lookup[4][out] = 45;

        return seg_lookup;
    }

    void init(const ConfigReader& cfg) {
        LOUT("Exec TrajectoryGenerator init()" << std::endl);

        std::string file_path;
        cfg.get("TrajectoryGenerator::segments_file", file_path);

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
                // data line, add to segment
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
                LOUT("ERROR: something wrong with the segments file" << std::endl);
        }

        LOUT("Number of Segments: " << segments.size() << std::endl);

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

        //TODO: implement method for finding the starting segment and curve
        segment_index = 13;
        curve_index = 0;
    }

    void execute() {
        //LOUT("Current mode on BBOARD: " << (int)BBOARD->getDrivingMode().current_mode << std::endl);

        try{
            while (true) {
                State state = BBOARD->getState();

                if (segments[segment_index].get(curve_index).reached_end(state.position)) {
                    if (curve_index < segments[segment_index].size() - 1) {
                        curve_index++;
                    }
                    else {
                        curve_index = 0;
                        DrivingCommand next_command = commands.front();
                        commands.pop_front();

                        int end_node = segment_index % 10;
                        segment_index = seg_lookup[end_node][next_command];

                        LOUT("Start segment " << segment_index << std::endl);
                    }

                    ReferenceTrajectory rt;
                    rt.path = segments[segment_index].get(curve_index);
                    BBOARD->setReferenceTrajectory(rt);

                    LOUT("Curve index: " << curve_index << std::endl);
                }
                else {
                    // do nothing
                }

                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
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
