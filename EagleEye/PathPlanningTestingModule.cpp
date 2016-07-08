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

    public:
        PathPlanningTestingModule () {;}
        ~PathPlanningTestingModule () {;}


	void init(const ConfigReader& cfg) {
        planner = PathPlanning(cfg);
	}

        void execute () {
          try{

            while (true) {


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


