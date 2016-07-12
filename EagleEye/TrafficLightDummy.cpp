#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Vec.h"
#include "DataObjects.h"

#include <iostream>
#include <sstream>
#include <string>

using namespace std;

namespace DerWeg {

  /** TrafficLightDummy */
  class TrafficLightDummy : public KogmoThread {

    private:
        Vec position;

        double cycle_duration;
        double red_percentage;
    public:
        TrafficLightDummy () {;}
        ~TrafficLightDummy () {;}


	void init(const ConfigReader& cfg) {
        cfg.get("TrafficLightDummy::cycle_duration", cycle_duration);
        cfg.get("TrafficLightDummy::red_percentage", red_percentage);

        std::vector<double> intersec_midpoint_temp;
        cfg.get("TrajectoryGenerator::intersection_midpoint", intersec_midpoint_temp);
        position.x = intersec_midpoint_temp[0];
        position.y = intersec_midpoint_temp[1];
	}

        void execute () {
          try{

            int counter = 0;
            int wait_ms = 100;
            int counts_per_second = 1000 / wait_ms;
            int counts_per_cycle = cycle_duration * counts_per_second;

            while (true) {
                counter = (counter + 1) % counts_per_cycle;
                TrafficLightData tl_data;

                double radius = 200;
                double t = 2*M_PI * counter / counts_per_cycle;
                double x = radius * std::cos(t);
                double y = radius * std::sin(t);

                tl_data.position = position;// + Vec(x, y);

                // Plot measured position as red dot in AnicarViewer
                std::stringstream pos;

                State state = BBOARD->getState();

                double scalar_prod = (position - state.sg_position) * Vec(1,0).rotate(state.orientation);
                //LOUT("sp = " << scalar_prod << "\n");

                bool passed_tl = (BBOARD->getReferenceTrajectory().path.behind_intersec || scalar_prod < 0);

                if (passed_tl) {
                    //LOUT("BLUE\n");
                    pos << "thick blue dot ";
                    tl_data.state = none;
                } else if(counter < red_percentage * counts_per_cycle) {
                    //LOUT("RED\n");
                    pos << "thick red dot ";
                    tl_data.state = red;
                } else {
                    //LOUT("GREEN\n");
                    pos << "thick green dot ";
                    tl_data.state = green;
                }

                //LOUT("Write tl_data to blackboard, state = " << tl_data.state <<std::endl);
                BBOARD->setTrafficLight(tl_data);

                pos << tl_data.position.x << " " << tl_data.position.y << std::endl;
                BBOARD->addPlotCommand(pos.str());

                boost::this_thread::sleep(boost::posix_time::milliseconds(wait_ms));
                boost::this_thread::interruption_point();
            }
          }catch(boost::thread_interrupted&){;}
        }

  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::TrafficLightDummy> application ("TrafficLightDummy");

}

