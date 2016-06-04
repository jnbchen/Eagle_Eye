#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Vec.h"

namespace DerWeg {

  /** StateEstimator */
  class StateEstimator : public KogmoThread {
  private:
    double center_offset;
  public:
    StateEstimator () {;}
    ~StateEstimator () {;}

    void init(const ConfigReader& cfg) {
      cfg.get ("StateEstimator::center_offset", center_offset);
    }

    void execute () {
      try{
        while (true) {

            State state;
            Pose pose = BBOARD->getVehiclePose();
            Odometry odometry = BBOARD->getOdometry();

            //Transform position from stargazer to center of mass
            state.position = pose.position - 500 * Vec(1,0).rotate(pose.orientation);
            //state.position = pose.position;
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
