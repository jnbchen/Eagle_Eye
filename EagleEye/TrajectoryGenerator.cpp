#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

namespace DerWeg {

  /** TrajectoryGenerator */
  class TrajectoryGenerator : public KogmoThread {
  public:
    TrajectoryGenerator () {;}
    ~TrajectoryGenerator () {;}
    void execute () {
      try{
        while (true) {

          // to be implemented ...

          //boost::this_thread::sleep(boost::posix_time::milliseconds(20));
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
