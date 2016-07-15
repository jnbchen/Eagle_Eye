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

  /** PathPlanningVirtualCone */
  class PathPlanningVirtualCone : public KogmoThread {

    private:
        PathPlanning planner;
        Vec intersection;

    public:
        PathPlanningVirtualCone () {;}
        ~PathPlanningVirtualCone () {;}


	void init(const ConfigReader& cfg) {
        planner = PathPlanning(cfg);

        std::vector<double> intersec_midpoint_temp;
        cfg.get("TrajectoryGenerator::intersection_midpoint", intersec_midpoint_temp);
        intersection.x = intersec_midpoint_temp[0];
        intersection.y = intersec_midpoint_temp[1] + 1200;
	}

        void execute () {
          try{
            double coneRadius = 100;
            vector<Circle> obstacles;
            obstacles.push_back(Circle(Vec(4000,3632), 100));
            obstacles.push_back(Circle(Vec(3701,3636), 100));
            obstacles.push_back(Circle(Vec(3403,3628), 100));
            obstacles.push_back(Circle(Vec(3064,3623), 100));
            obstacles.push_back(Circle(Vec(2746,3621), 100));
            obstacles.push_back(Circle(Vec(2420,3593), 100));
            obstacles.push_back(Circle(Vec(2070,3483), 100));
            obstacles.push_back(Circle(Vec(1845,3275), 100));
            obstacles.push_back(Circle(Vec(1634,2943), 100));
            obstacles.push_back(Circle(Vec(1573,2590), 100));
            obstacles.push_back(Circle(Vec(1569,2262), 100));
            obstacles.push_back(Circle(Vec(1567,1930), 100));
            obstacles.push_back(Circle(Vec(1580,1559), 100));
            obstacles.push_back(Circle(Vec(1620,1223), 100));
            obstacles.push_back(Circle(Vec(1750,893), 100));
            obstacles.push_back(Circle(Vec(2007,603), 100));
            obstacles.push_back(Circle(Vec(2288,389), 100));
            obstacles.push_back(Circle(Vec(2690,283), 100));
            obstacles.push_back(Circle(Vec(3147,265), 100));
            obstacles.push_back(Circle(Vec(3542,267), 100));

            obstacles.push_back(Circle(Vec(4008,2912), 100));
            obstacles.push_back(Circle(Vec(3694,2910), 100));
            obstacles.push_back(Circle(Vec(3390,2906), 100));
            obstacles.push_back(Circle(Vec(3060,2902), 100));
            obstacles.push_back(Circle(Vec(2720,2892), 100));
            obstacles.push_back(Circle(Vec(2412,2661), 100));
            obstacles.push_back(Circle(Vec(2351,2288), 100));
            obstacles.push_back(Circle(Vec(2355,1913), 100));
            obstacles.push_back(Circle(Vec(2387,1496), 100));
            obstacles.push_back(Circle(Vec(2523,1166), 100));
            obstacles.push_back(Circle(Vec(2869,996), 100));
            obstacles.push_back(Circle(Vec(3216,988), 100));
            obstacles.push_back(Circle(Vec(3520,992), 100));
            //obstacles.push_back(Circle(Vec(,), 100));




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
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::PathPlanningVirtualCone> application ("PathPlanningVirtualCone");

}



