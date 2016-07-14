#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Vec.h"
#include "DataObjects.h"

#include "PathPlanning.h"


#include <iostream>
#include <sstream>
#include <string>

using namespace std;

namespace DerWeg {

  /** PathPlanningTestingModule */
  class PathPlanningTestingModule : public KogmoThread {

    private:
        PathPlanning planner;
        Vec intersection;

    public:
        PathPlanningTestingModule () {;}
        ~PathPlanningTestingModule () {;}


	void init(const ConfigReader& cfg) {
        planner = PathPlanning(cfg);

        std::vector<double> intersec_midpoint_temp;
        cfg.get("TrajectoryGenerator::intersection_midpoint", intersec_midpoint_temp);
        intersection.x = intersec_midpoint_temp[0];
        intersection.y = intersec_midpoint_temp[1];
	}

        void execute () {
          try{
            vector<Circle> obstacles;
            double r1 = 3000;
            double r2 = 4500;
            int obstacle_count = 20;
            for (int i=0; i<obstacle_count; i++) {
                Angle alpha = Angle::rad_angle(i * M_PI / obstacle_count);
                obstacles.push_back(Circle(intersection + r1 * Vec(1,0).rotate(alpha), 150));
                obstacles.push_back(Circle(intersection + r2 * Vec(1,0).rotate(alpha), 150));
            }

            while (true) {

                Velocity dv = BBOARD->getDesiredVelocity();
                dv.steer = planner.findPath(obstacles).steer;



                BBOARD->setDesiredVelocity(dv);

                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                boost::this_thread::interruption_point();
            }
          }catch(boost::thread_interrupted&){;}
        }

  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::PathPlanningTestingModule> application ("PathPlanningTestingModule");

}


