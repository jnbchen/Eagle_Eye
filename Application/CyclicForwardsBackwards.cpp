
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include <cmath>

namespace DerWeg {

  /** eine sehr einfache Beispielanwendung, die das Fahrzeug zyklisch vorwaerts und rueckwaerts fahren laesst */
  class CyclicForwardsBackwards : public KogmoThread {
  public:
    CyclicForwardsBackwards () {;}
    ~CyclicForwardsBackwards () {;}
    void execute () {
      try{
        Timestamp tinit;
        while (true) {
          Velocity dv;
          double dt = static_cast<double>(tinit.elapsed_msec())/4000*2*M_PI;
          dv.velocity = 0.8*std::sin(dt);
          dv.steer=Angle::deg_angle (std::sin(dt)>0 ? +20 : -20);
          BBOARD->setDesiredVelocity(dv);

          boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::CyclicForwardsBackwards> application ("CyclicForwardsBackwards");

}
