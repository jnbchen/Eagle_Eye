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
        while (true) {

            State state;
            Pose pose = BBOARD->getVehiclePose();
            Odometry odometry = BBOARD->getOdometry();

            state.position = pose.position;
            state.stddev = pose.stddev;
            state.orientation = pose.orientation;
            state.velocity = pose.velocity;
            state.yawrate = pose.yawrate;
            state.timestamp = pose.timestamp;

            state.velocity_tire = odometry.velocity;
            state.steer = odometry.steer;

            BBOARD->setState(state);

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
