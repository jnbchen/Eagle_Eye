#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

namespace DerWeg {

  /** A very simple example and testing application that moves
      the vehicle following a wavy line*/
  class DriveWigglyLines : public KogmoThread {
  public:
    DriveWigglyLines () {;}
    ~DriveWigglyLines () {;}
    void execute () {
      try{
        while (true) {
          Pose vp;
          vp = BBOARD->getVehiclePose ();
          double deltay = vp.position.y-2000;
          Velocity dv;
          dv.velocity = (abs(deltay)>1500 ? 0 : 0.5);
          double sign = vp.orientation.in_between(Angle::deg_angle(-90),
                                       Angle::deg_angle(90)) ? -1 : +1;
          dv.steer=Angle::deg_angle (sign*deltay/40);
          BBOARD->setDesiredVelocity(dv);

          boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {
  // Registers the plugin in the factory
  static DerWeg::PluginBuilder<DerWeg::KogmoThread,
     DerWeg::DriveWigglyLines> application ("DriveWigglyLines");
}