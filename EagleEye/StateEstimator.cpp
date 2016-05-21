#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

namespace DerWeg {

  /** StateEstimator */
  class StateEstimator : public KogmoThread {
  public:
    StateEstimator () {;}
    ~StateEstimator () {;}
    void execute () {
      try{
        while (BBOARD->getActive()) {
          logfiel<<BBOARD->getVehiclePose().position.x << '\t'
                 << BBOARD->getVehiclePose().position.y << '\t'
                 << BBOARD->getVehiclePose().orientation.get_deg() << '\t'
                 << BBOARD->getOdometry().velocity << '\t'
                 << BBOARD->getOdometry().steer.get_deg_180() << '\t'
                 << endl;
          //wieso gibt hier keine init() oder deinit()? bitte prüfen Sie.

          //boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::StateEstimator> application ("StateEstimator");

}
