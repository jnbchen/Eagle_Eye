
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

namespace DerWeg {

  class MotorControlDummy : public KogmoThread {
  public:
    void execute () {
      try {
        while (true) {
          Velocity dv = BBOARD->getDesiredVelocity();
          Odometry odo;
          if (BBOARD->getActive()) {
            odo.velocity=dv.velocity;
            odo.steer=dv.steer;
            if (odo.velocity>2.5) odo.velocity=2.5;
            if (odo.velocity<-2.5) odo.velocity=-2.5;
            if (odo.steer.get_deg_180()>35) odo.steer.set_deg(35);
            if (odo.steer.get_deg_180()<-35) odo.steer.set_deg(-35);
          } else {
            odo.velocity=0;
            odo.steer=Angle::zero;
          }
          BBOARD->setOdometry(odo);

          BBOARD->waitForDesiredVelocity();
          boost::this_thread::interruption_point();
        }
      }
      catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::MotorControlDummy> motorcontrol_dummy ("MotorDummy");

} // namespace
